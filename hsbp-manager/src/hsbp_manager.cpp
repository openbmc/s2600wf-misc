/*
// Copyright (c) 2019 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
*/

#include "utils.hpp"

#include <boost/algorithm/string/replace.hpp>
#include <boost/asio/steady_timer.hpp>
#include <iostream>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/bus/match.hpp>
#include <string>
#include <utility>

extern "C" {
#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
}

constexpr const char* configType =
    "xyz.openbmc_project.Configuration.Intel_HSBP_CPLD";

constexpr size_t scanRateSeconds = 5;
constexpr size_t maxDrives = 8; // only 1 byte alloted

boost::asio::io_context io;
auto conn = std::make_shared<sdbusplus::asio::connection>(io);
sdbusplus::asio::object_server objServer(conn);

static std::string zeroPad(const uint8_t val)
{
    std::ostringstream version;
    version << std::setw(2) << std::setfill('0') << static_cast<size_t>(val);
    return version.str();
}

struct Drive
{
    Drive(size_t driveIndex, bool isPresent, bool isOperational, bool nvme,
          bool rebuilding) :
        isNvme(nvme)
    {
        constexpr const char* basePath =
            "/xyz/openbmc_project/inventory/item/drive/Drive_";
        itemIface = objServer.add_interface(
            basePath + std::to_string(driveIndex), inventory::interface);
        itemIface->register_property("Present", isPresent);
        itemIface->register_property("PrettyName",
                                     "Drive " + std::to_string(driveIndex));
        itemIface->initialize();
        operationalIface = objServer.add_interface(
            itemIface->get_object_path(),
            "xyz.openbmc_project.State.Decorator.OperationalStatus");
        operationalIface->register_property("Functional", isOperational);
        operationalIface->initialize();
        rebuildingIface = objServer.add_interface(
            itemIface->get_object_path(), "xyz.openbmc_project.State.Drive");
        rebuildingIface->register_property("Rebuilding", rebuilding);
        rebuildingIface->initialize();
    }
    ~Drive()
    {
        objServer.remove_interface(itemIface);
        objServer.remove_interface(operationalIface);
        objServer.remove_interface(rebuildingIface);
    }

    std::shared_ptr<sdbusplus::asio::dbus_interface> itemIface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> operationalIface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> rebuildingIface;
    bool isNvme;
};

struct Backplane
{

    Backplane(size_t busIn, size_t addressIn, size_t backplaneIndexIn,
              const std::string& nameIn) :
        bus(busIn),
        address(addressIn), backplaneIndex(backplaneIndexIn - 1), name(nameIn),
        timer(std::make_shared<boost::asio::steady_timer>(io))
    {
    }
    void run()
    {
        file = open(("/dev/i2c-" + std::to_string(bus)).c_str(), O_RDWR);
        if (file < 0)
        {
            std::cerr << "unable to open bus " << bus << "\n";
            return;
        }

        if (ioctl(file, I2C_SLAVE_FORCE, address) < 0)
        {
            std::cerr << "unable to set address to " << address << "\n";
            return;
        }

        if (!getPresent())
        {
            std::cerr << "Cannot detect CPLD\n";
            return;
        }

        getBootVer(bootVer);
        getFPGAVer(fpgaVer);
        getSecurityRev(securityRev);
        std::string dbusName = boost::replace_all_copy(name, " ", "_");
        hsbpItemIface = objServer.add_interface(
            "/xyz/openbmc_project/inventory/item/hsbp/" + dbusName,
            inventory::interface);
        hsbpItemIface->register_property("Present", true);
        hsbpItemIface->register_property("PrettyName", name);
        hsbpItemIface->initialize();

        versionIface =
            objServer.add_interface(hsbpItemIface->get_object_path(),
                                    "xyz.openbmc_project.Software.Version");
        versionIface->register_property("Version", zeroPad(bootVer) + "." +
                                                       zeroPad(fpgaVer) + "." +
                                                       zeroPad(securityRev));
        versionIface->register_property(
            "Purpose",
            std::string(
                "xyz.openbmc_project.Software.Version.VersionPurpose.HSBP"));
        versionIface->initialize();
        getPresence(presence);
        getIFDET(ifdet);

        createDrives();

        runTimer();
    }

    void runTimer()
    {
        timer->expires_after(std::chrono::seconds(scanRateSeconds));
        timer->async_wait([this](boost::system::error_code ec) {
            if (ec == boost::asio::error::operation_aborted)
            {
                // we're being destroyed
                return;
            }
            else if (ec)
            {
                std::cerr << "timer error " << ec.message() << "\n";
                return;
            }
            uint8_t curPresence = 0;
            uint8_t curIFDET = 0;
            uint8_t curFailed = 0;
            uint8_t curRebuild = 0;

            getPresence(curPresence);
            getIFDET(curIFDET);
            getFailed(curFailed);
            getRebuild(curRebuild);

            if (curPresence != presence || curIFDET != ifdet ||
                curFailed != failed || curRebuild != rebuilding)
            {
                presence = curPresence;
                ifdet = curIFDET;
                failed = curFailed;
                rebuilding = curRebuild;
                updateDrives();
            }
            runTimer();
        });
    }

    void createDrives()
    {
        uint8_t nvme = ifdet ^ presence;
        for (size_t ii = 0; ii < maxDrives; ii++)
        {
            bool isNvme = nvme & (1 << ii);
            bool isPresent = isNvme || (presence & (1 << ii));
            bool isFailed = !isPresent || failed & (1 << ii);
            bool isRebuilding = !isPresent && (rebuilding & (1 << ii));

            // +1 to convert from 0 based to 1 based
            size_t driveIndex = (backplaneIndex * maxDrives) + ii + 1;
            drives.emplace_back(driveIndex, isPresent, !isFailed, isNvme,
                                isRebuilding);
        }
    }

    void updateDrives()
    {

        uint8_t nvme = ifdet ^ presence;
        for (size_t ii = 0; ii < maxDrives; ii++)
        {
            bool isNvme = nvme & (1 << ii);
            bool isPresent = isNvme || (presence & (1 << ii));
            bool isFailed = !isPresent || (failed & (1 << ii));
            bool isRebuilding = isPresent && (rebuilding & (1 << ii));

            Drive& drive = drives[ii];
            drive.isNvme = isNvme;
            drive.itemIface->set_property("Present", isPresent);
            drive.operationalIface->set_property("Functional", !isFailed);
            drive.rebuildingIface->set_property("Rebuilding", isRebuilding);
        }
    }

    bool getPresent()
    {
        present = i2c_smbus_read_byte(file) >= 0;
        return present;
    }

    bool getTypeID(uint8_t& val)
    {
        constexpr uint8_t addr = 2;
        int ret = i2c_smbus_read_byte_data(file, addr);
        if (ret < 0)
        {
            std::cerr << "Error " << __FUNCTION__ << "\n";
            return false;
        }
        val = static_cast<uint8_t>(ret);
        return true;
    }

    bool getBootVer(uint8_t& val)
    {
        constexpr uint8_t addr = 3;
        int ret = i2c_smbus_read_byte_data(file, addr);
        if (ret < 0)
        {
            std::cerr << "Error " << __FUNCTION__ << "\n";
            return false;
        }
        val = static_cast<uint8_t>(ret);
        return true;
    }

    bool getFPGAVer(uint8_t& val)
    {
        constexpr uint8_t addr = 4;
        int ret = i2c_smbus_read_byte_data(file, addr);
        if (ret < 0)
        {
            std::cerr << "Error " << __FUNCTION__ << "\n";
            return false;
        }
        val = static_cast<uint8_t>(ret);
        return true;
    }

    bool getSecurityRev(uint8_t& val)
    {
        constexpr uint8_t addr = 5;
        int ret = i2c_smbus_read_byte_data(file, addr);
        if (ret < 0)
        {
            std::cerr << "Error " << __FUNCTION__ << "\n";
            return false;
        }
        val = static_cast<uint8_t>(ret);
        return true;
    }

    bool getPresence(uint8_t& val)
    {
        // NVMe drives do not assert PRSNTn, and as such do not get reported as
        // PRESENT in this register

        constexpr uint8_t addr = 8;

        int ret = i2c_smbus_read_byte_data(file, addr);
        if (ret < 0)
        {
            std::cerr << "Error " << __FUNCTION__ << "\n";
            return false;
        }
        // presence is inverted
        val = static_cast<uint8_t>(~ret);
        return true;
    }

    bool getIFDET(uint8_t& val)
    {
        // This register is a bitmap of parallel GPIO pins connected to the
        // IFDETn pin of a drive slot. SATA, SAS, and NVMe drives all assert
        // IFDETn low when they are inserted into the HSBP.This register, in
        // combination with the PRESENCE register, are used by the BMC to detect
        // the presence of NVMe drives.

        constexpr uint8_t addr = 9;

        int ret = i2c_smbus_read_byte_data(file, addr);
        if (ret < 0)
        {
            std::cerr << "Error " << __FUNCTION__ << "\n";
            return false;
        }
        // ifdet is inverted
        val = static_cast<uint8_t>(~ret);
        return true;
    }

    bool getFailed(uint8_t& val)
    {
        constexpr uint8_t addr = 0xC;
        int ret = i2c_smbus_read_byte_data(file, addr);
        if (ret < 0)
        {
            std::cerr << "Error " << __FUNCTION__ << "\n";
            return false;
        }
        val = static_cast<uint8_t>(ret);
        return true;
    }

    bool getRebuild(uint8_t& val)
    {
        constexpr uint8_t addr = 0xD;
        int ret = i2c_smbus_read_byte_data(file, addr);
        if (ret < 0)
        {
            std::cerr << "Error " << __FUNCTION__ << "\n";
            return false;
        }
        val = static_cast<uint8_t>(ret);
        return true;
    }

    ~Backplane()
    {
        objServer.remove_interface(hsbpItemIface);
        objServer.remove_interface(versionIface);
        if (file >= 0)
        {
            close(file);
        }
    }

    size_t bus;
    size_t address;
    size_t backplaneIndex;
    std::string name;
    std::shared_ptr<boost::asio::steady_timer> timer;
    bool present = false;
    uint8_t typeId = 0;
    uint8_t bootVer = 0;
    uint8_t fpgaVer = 0;
    uint8_t securityRev = 0;
    uint8_t funSupported = 0;
    uint8_t presence = 0;
    uint8_t ifdet = 0;
    uint8_t failed = 0;
    uint8_t rebuilding = 0;

    int file = -1;

    std::string type;

    std::shared_ptr<sdbusplus::asio::dbus_interface> hsbpItemIface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> versionIface;

    std::vector<Drive> drives;
};

std::unordered_map<std::string, Backplane> backplanes;

void populate()
{
    conn->async_method_call(
        [](const boost::system::error_code ec, const GetSubTreeType& subtree) {
            if (ec)
            {
                std::cerr << "Error contacting mapper " << ec.message() << "\n";
                return;
            }
            for (const auto& [path, objDict] : subtree)
            {
                if (objDict.empty())
                {
                    continue;
                }

                const std::string& owner = objDict.begin()->first;
                conn->async_method_call(
                    [path](const boost::system::error_code ec2,
                           const boost::container::flat_map<
                               std::string, BasicVariantType>& resp) {
                        if (ec2)
                        {
                            std::cerr << "Error Getting Config "
                                      << ec2.message() << "\n";
                            return;
                        }
                        backplanes.clear();
                        std::optional<size_t> bus;
                        std::optional<size_t> address;
                        std::optional<size_t> backplaneIndex;
                        std::optional<std::string> name;
                        for (const auto& [key, value] : resp)
                        {
                            if (key == "Bus")
                            {
                                bus = std::get<uint64_t>(value);
                            }
                            else if (key == "Address")
                            {
                                address = std::get<uint64_t>(value);
                            }
                            else if (key == "Index")
                            {
                                backplaneIndex = std::get<uint64_t>(value);
                            }
                            else if (key == "Name")
                            {
                                name = std::get<std::string>(value);
                            }
                        }
                        if (!bus || !address || !name || !backplaneIndex)
                        {
                            std::cerr << "Illegal configuration at " << path
                                      << "\n";
                            return;
                        }
                        const auto& [backplane, status] = backplanes.emplace(
                            *name,
                            Backplane(*bus, *address, *backplaneIndex, *name));
                        backplane->second.run();
                    },
                    owner, path, "org.freedesktop.DBus.Properties", "GetAll",
                    configType);
            }
        },
        mapper::busName, mapper::path, mapper::interface, mapper::subtree, "/",
        0, std::array<const char*, 1>{configType});
}

int main()
{
    boost::asio::steady_timer callbackTimer(io);

    conn->request_name("xyz.openbmc_project.HsbpManager");

    sdbusplus::bus::match::match match(
        *conn,
        "type='signal',member='PropertiesChanged',arg0='" +
            std::string(configType) + "'",
        [&callbackTimer](sdbusplus::message::message&) {
            callbackTimer.expires_after(std::chrono::seconds(2));
            callbackTimer.async_wait([](const boost::system::error_code ec) {
                if (ec == boost::asio::error::operation_aborted)
                {
                    // timer was restarted
                    return;
                }
                else if (ec)
                {
                    std::cerr << "Timer error" << ec.message() << "\n";
                    return;
                }
                populate();
            });
        });

    io.post([]() { populate(); });
    io.run();
}
