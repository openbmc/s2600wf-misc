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

#include <bitset>
#include <boost/algorithm/string/replace.hpp>
#include <boost/asio/posix/stream_descriptor.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/container/flat_set.hpp>
#include <filesystem>
#include <fstream>
#include <gpiod.hpp>
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
constexpr const char* busName = "xyz.openbmc_project.HsbpManager";

constexpr size_t scanRateSeconds = 5;
constexpr size_t maxDrives = 8; // only 1 byte alloted

boost::asio::io_context io;
auto conn = std::make_shared<sdbusplus::asio::connection>(io);
sdbusplus::asio::object_server objServer(conn);

// GPIO Lines and Event Descriptors
static gpiod::line nvmeLvc3AlertLine;
static boost::asio::posix::stream_descriptor nvmeLvc3AlertEvent(io);

static std::string zeroPad(const uint8_t val)
{
    std::ostringstream version;
    version << std::setw(2) << std::setfill('0') << static_cast<size_t>(val);
    return version.str();
}

struct Mux
{
    Mux(size_t busIn, size_t addressIn, size_t channelsIn, size_t indexIn) :
        bus(busIn), address(addressIn), channels(channelsIn), index(indexIn)
    {
    }
    size_t bus;
    size_t address;
    size_t channels;
    size_t index;

    // to sort in the flat set
    bool operator<(const Mux& rhs) const
    {
        return index < rhs.index;
    }
};

enum class BlinkPattern : uint8_t
{
    off = 0x0,
    error = 0x2,
    terminate = 0x3
};

struct Led : std::enable_shared_from_this<Led>
{
    // led pattern addresses start at 0x10
    Led(const std::string& path, size_t index, int fd) :
        address(static_cast<uint8_t>(index + 0x10)), file(fd),
        ledInterface(objServer.add_interface(path, ledGroup::interface))
    {
        if (index >= maxDrives)
        {
            throw std::runtime_error("Invalid drive index");
        }

        if (!set(BlinkPattern::off))
        {
            std::cerr << "Cannot initialize LED " << path << "\n";
        }
    }

    // this has to be called outside the constructor for shared_from_this to
    // work
    void createInterface(void)
    {
        std::shared_ptr<Led> self = shared_from_this();

        ledInterface->register_property(
            ledGroup::asserted, false, [self](const bool req, bool& val) {
                if (req == val)
                {
                    return 1;
                }

                if (!isPowerOn())
                {
                    std::cerr << "Can't change blink state when power is off\n";
                    throw std::runtime_error(
                        "Can't change blink state when power is off");
                }
                BlinkPattern pattern =
                    req ? BlinkPattern::error : BlinkPattern::terminate;
                if (!self->set(pattern))
                {
                    std::cerr << "Can't change blink pattern\n";
                    throw std::runtime_error("Cannot set blink pattern");
                }
                val = req;
                return 1;
            });
        ledInterface->initialize();
    }

    virtual ~Led()
    {
        objServer.remove_interface(ledInterface);
    }

    bool set(BlinkPattern pattern)
    {
        int ret = i2c_smbus_write_byte_data(file, address,
                                            static_cast<uint8_t>(pattern));
        return ret >= 0;
    }

    uint8_t address;
    int file;
    std::shared_ptr<sdbusplus::asio::dbus_interface> ledInterface;
};

struct Drive
{
    Drive(size_t driveIndex, bool present, bool isOperational, bool nvme,
          bool rebuilding) :
        isNvme(nvme),
        isPresent(present), index(driveIndex)
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

        operationalIface->register_property(
            "Functional", isOperational,
            [this](const bool req, bool& property) {
                if (!isPresent)
                {
                    return 0;
                }
                if (property == req)
                {
                    return 1;
                }
                property = req;
                if (req)
                {
                    clearFailed();
                    return 1;
                }
                markFailed();
                return 1;
            });

        operationalIface->initialize();
        rebuildingIface = objServer.add_interface(
            itemIface->get_object_path(), "xyz.openbmc_project.State.Drive");
        rebuildingIface->register_property("Rebuilding", rebuilding);
        rebuildingIface->initialize();
        driveIface =
            objServer.add_interface(itemIface->get_object_path(),
                                    "xyz.openbmc_project.Inventory.Item.Drive");
        driveIface->initialize();
        associations = objServer.add_interface(itemIface->get_object_path(),
                                               association::interface);
        associations->register_property("Associations",
                                        std::vector<Association>{});
        associations->initialize();

        if (isPresent && (!isOperational || rebuilding))
        {
            markFailed();
        }
    }
    virtual ~Drive()
    {
        objServer.remove_interface(itemIface);
        objServer.remove_interface(operationalIface);
        objServer.remove_interface(rebuildingIface);
        objServer.remove_interface(assetIface);
        objServer.remove_interface(driveIface);
        objServer.remove_interface(associations);
    }

    void removeAsset()
    {
        objServer.remove_interface(assetIface);
        assetIface = nullptr;
    }

    void createAsset(
        const boost::container::flat_map<std::string, std::string>& data)
    {
        if (assetIface != nullptr)
        {
            return;
        }
        assetIface = objServer.add_interface(
            itemIface->get_object_path(),
            "xyz.openbmc_project.Inventory.Decorator.Asset");
        for (const auto& [key, value] : data)
        {
            assetIface->register_property(key, value);
            if (key == "SerialNumber")
            {
                serialNumber = value;
                serialNumberInitialized = true;
            }
        }
        assetIface->initialize();
    }

    void markFailed(void)
    {
        // todo: maybe look this up via mapper
        constexpr const char* globalInventoryPath =
            "/xyz/openbmc_project/CallbackManager";

        if (!isPresent)
        {
            return;
        }

        operationalIface->set_property("Functional", false);
        std::vector<Association> warning = {
            {"", "warning", globalInventoryPath}};
        associations->set_property("Associations", warning);
        logDriveError("Drive " + std::to_string(index));
    }

    void clearFailed(void)
    {
        operationalIface->set_property("Functional", true);
        associations->set_property("Associations", std::vector<Association>{});
    }

    void setPresent(bool set)
    {
        // nvme drives get detected by their fru
        if (set == isPresent)
        {
            return;
        }
        itemIface->set_property("Present", set);
        isPresent = set;
    }

    void logPresent()
    {
        if (isNvme && !serialNumberInitialized)
        {
            // wait until NVMe asset is updated to include the serial number
            // from the NVMe drive
            return;
        }

        if (!isPresent && loggedPresent)
        {
            loggedPresent = false;
            logDeviceRemoved("Drive", std::to_string(index), serialNumber);
            serialNumber = "N/A";
            serialNumberInitialized = false;
            removeAsset();
        }
        else if (isPresent && !loggedPresent)
        {
            loggedPresent = true;
            logDeviceAdded("Drive", std::to_string(index), serialNumber);
        }
    }

    std::shared_ptr<sdbusplus::asio::dbus_interface> itemIface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> operationalIface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> rebuildingIface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> assetIface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> driveIface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> associations;

    bool isNvme;
    bool isPresent;
    size_t index;
    std::string serialNumber = "N/A";
    bool serialNumberInitialized = false;
    bool loggedPresent = false;
};

struct Backplane : std::enable_shared_from_this<Backplane>
{

    Backplane(size_t busIn, size_t addressIn, size_t backplaneIndexIn,
              const std::string& nameIn) :
        bus(busIn),
        address(addressIn), backplaneIndex(backplaneIndexIn - 1), name(nameIn),
        timer(boost::asio::steady_timer(io)),
        muxes(std::make_shared<boost::container::flat_set<Mux>>())
    {
    }
    void populateAsset(const std::string& rootPath, const std::string& busname)
    {
        conn->async_method_call(
            [assetIface{assetInterface}, hsbpIface{hsbpItemIface}](
                const boost::system::error_code ec,
                const boost::container::flat_map<
                    std::string, std::variant<std::string>>& values) mutable {
                if (ec)
                {
                    std::cerr
                        << "Error getting asset tag from HSBP configuration\n";

                    return;
                }
                assetIface = objServer.add_interface(
                    hsbpIface->get_object_path(), assetTag);
                for (const auto& [key, value] : values)
                {
                    const std::string* ptr = std::get_if<std::string>(&value);
                    if (ptr == nullptr)
                    {
                        std::cerr << key << " Invalid type!\n";
                        continue;
                    }
                    assetIface->register_property(key, *ptr);
                }
                assetIface->initialize();
            },
            busname, rootPath, "org.freedesktop.DBus.Properties", "GetAll",
            assetTag);
    }

    void run(const std::string& rootPath, const std::string& busname)
    {
        file = open(("/dev/i2c-" + std::to_string(bus)).c_str(),
                    O_RDWR | O_CLOEXEC);
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

        storageInterface = objServer.add_interface(
            hsbpItemIface->get_object_path(),
            "xyz.openbmc_project.Inventory.Item.StorageController");
        storageInterface->initialize();

        versionIface =
            objServer.add_interface("/xyz/openbmc_project/software/" + dbusName,
                                    "xyz.openbmc_project.Software.Version");
        versionIface->register_property("Version", zeroPad(bootVer) + "." +
                                                       zeroPad(fpgaVer) + "." +
                                                       zeroPad(securityRev));
        versionIface->register_property(
            "Purpose",
            std::string(
                "xyz.openbmc_project.Software.Version.VersionPurpose.HSBP"));
        versionIface->initialize();

        auto activationIface =
            objServer.add_interface("/xyz/openbmc_project/software/" + dbusName,
                                    "xyz.openbmc_project.Software.Activation");

        activationIface->register_property(
            "Activation",
            std::string(
                "xyz.openbmc_project.Software.Activation.Activations.Active"));
        activationIface->register_property(
            "RequestedActivation",
            std::string("xyz.openbmc_project.Software.Activation."
                        "RequestedActivations.None"));

        activationIface->initialize();

        getPresence(presence);
        getIFDET(ifdet);

        populateAsset(rootPath, busname);

        createDrives();

        runTimer();
    }

    void runTimer()
    {
        timer.expires_after(std::chrono::seconds(scanRateSeconds));
        timer.async_wait([weak{std::weak_ptr<Backplane>(shared_from_this())}](
                             boost::system::error_code ec) {
            auto self = weak.lock();
            if (!self)
            {
                return;
            }
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

            if (!isPowerOn())
            {
                // can't access hsbp when power is off
                self->runTimer();
                return;
            }

            self->getPresence(self->presence);
            self->getIFDET(self->ifdet);
            self->getFailed(self->failed);
            self->getRebuild(self->rebuilding);

            self->updateDrives();
            self->runTimer();
        });
    }

    void createDrives()
    {
        for (size_t ii = 0; ii < maxDrives; ii++)
        {
            uint8_t driveSlot = (1 << ii);
            bool isNvme = ((ifdet & driveSlot) && !(presence & driveSlot));
            bool isPresent = isNvme || (presence & driveSlot);
            bool isFailed = !isPresent || failed & driveSlot;
            bool isRebuilding = !isPresent && (rebuilding & driveSlot);

            // +1 to convert from 0 based to 1 based
            size_t driveIndex = (backplaneIndex * maxDrives) + ii + 1;
            Drive& drive = drives.emplace_back(driveIndex, isPresent, !isFailed,
                                               isNvme, isRebuilding);
            std::shared_ptr<Led> led = leds.emplace_back(std::make_shared<Led>(
                drive.itemIface->get_object_path(), ii, file));
            led->createInterface();
        }
    }

    void updateDrives()
    {
        size_t ii = 0;

        for (auto it = drives.begin(); it != drives.end(); it++, ii++)
        {
            uint8_t driveSlot = (1 << ii);
            bool isNvme = ((ifdet & driveSlot) && !(presence & driveSlot));
            bool isPresent = isNvme || (presence & driveSlot);
            bool isFailed = !isPresent || (failed & driveSlot);
            bool isRebuilding = isPresent && (rebuilding & driveSlot);

            it->isNvme = isNvme;
            it->setPresent(isPresent);
            it->logPresent();

            it->rebuildingIface->set_property("Rebuilding", isRebuilding);
            if (isFailed || isRebuilding)
            {
                it->markFailed();
            }
            else
            {
                it->clearFailed();
            }
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
            std::cerr << "Error " << __FUNCTION__ << " " << strerror(ret)
                      << "\n";
            return false;
        }
        val = static_cast<uint8_t>(ret);
        return true;
    }

    virtual ~Backplane()
    {
        objServer.remove_interface(hsbpItemIface);
        objServer.remove_interface(versionIface);
        timer.cancel();
        if (file >= 0)
        {
            close(file);
        }
    }

    size_t bus;
    size_t address;
    size_t backplaneIndex;
    std::string name;
    boost::asio::steady_timer timer;
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
    std::shared_ptr<sdbusplus::asio::dbus_interface> storageInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> assetInterface;

    std::list<Drive> drives;
    std::vector<std::shared_ptr<Led>> leds;
    std::shared_ptr<boost::container::flat_set<Mux>> muxes;
};

std::unordered_map<std::string, std::shared_ptr<Backplane>> backplanes;
std::list<Drive> ownerlessDrives; // drives without a backplane

static size_t getDriveCount()
{
    size_t count = 0;
    for (const auto& [key, backplane] : backplanes)
    {
        count += backplane->drives.size();
    }
    return count + ownerlessDrives.size();
}

void updateAssets()
{
    static constexpr const char* nvmeType =
        "xyz.openbmc_project.Inventory.Item.NVMe";

    conn->async_method_call(
        [](const boost::system::error_code ec, const GetSubTreeType& subtree) {
            if (ec)
            {
                std::cerr << "Error contacting mapper " << ec.message() << "\n";
                return;
            }

            // drives may get an owner during this, or we might disover more
            // drives
            ownerlessDrives.clear();
            for (const auto& [path, objDict] : subtree)
            {
                if (objDict.empty())
                {
                    continue;
                }

                const std::string& owner = objDict.begin()->first;
                // we export this interface too
                if (owner == busName)
                {
                    continue;
                }
                if (std::find(objDict.begin()->second.begin(),
                              objDict.begin()->second.end(),
                              assetTag) == objDict.begin()->second.end())
                {
                    // no asset tag to associate to
                    continue;
                }

                conn->async_method_call(
                    [path](const boost::system::error_code ec2,
                           const boost::container::flat_map<
                               std::string,
                               std::variant<uint64_t, std::string>>& values) {
                        if (ec2)
                        {
                            std::cerr << "Error Getting Config "
                                      << ec2.message() << " " << __FUNCTION__
                                      << "\n";
                            return;
                        }
                        auto findBus = values.find("Bus");

                        if (findBus == values.end())
                        {
                            std::cerr << "Illegal interface at " << path
                                      << "\n";
                            return;
                        }

                        // find the mux bus and addr
                        size_t muxBus = static_cast<size_t>(
                            std::get<uint64_t>(findBus->second));
                        std::filesystem::path muxPath =
                            "/sys/bus/i2c/devices/i2c-" +
                            std::to_string(muxBus) + "/mux_device";
                        if (!std::filesystem::is_symlink(muxPath))
                        {
                            std::cerr << path << " mux does not exist\n";
                            return;
                        }

                        // we should be getting something of the form 7-0052
                        // for bus 7 addr 52
                        std::string fname =
                            std::filesystem::read_symlink(muxPath).filename();
                        auto findDash = fname.find('-');

                        if (findDash == std::string::npos ||
                            findDash + 1 >= fname.size())
                        {
                            std::cerr << path << " mux path invalid\n";
                            return;
                        }

                        std::string busStr = fname.substr(0, findDash);
                        std::string muxStr = fname.substr(findDash + 1);

                        size_t bus = static_cast<size_t>(std::stoi(busStr));
                        size_t addr =
                            static_cast<size_t>(std::stoi(muxStr, nullptr, 16));
                        size_t muxIndex = 0;

                        // find the channel of the mux the drive is on
                        std::ifstream nameFile("/sys/bus/i2c/devices/i2c-" +
                                               std::to_string(muxBus) +
                                               "/name");
                        if (!nameFile)
                        {
                            std::cerr << "Unable to open name file of bus "
                                      << muxBus << "\n";
                            return;
                        }

                        std::string nameStr;
                        std::getline(nameFile, nameStr);

                        // file is of the form "i2c-4-mux (chan_id 1)", get chan
                        // assume single digit chan
                        const std::string prefix = "chan_id ";
                        size_t findId = nameStr.find(prefix);
                        if (findId == std::string::npos ||
                            findId + 1 >= nameStr.size())
                        {
                            std::cerr << "Illegal name file on bus " << muxBus
                                      << "\n";
                        }

                        std::string indexStr =
                            nameStr.substr(findId + prefix.size(), 1);

                        size_t driveIndex = std::stoi(indexStr);

                        Backplane* parent = nullptr;
                        for (auto& [name, backplane] : backplanes)
                        {
                            muxIndex = 0;
                            for (const Mux& mux : *(backplane->muxes))
                            {
                                if (bus == mux.bus && addr == mux.address)
                                {
                                    parent = backplane.get();
                                    break;
                                }
                                muxIndex += mux.channels;
                            }
                        }
                        boost::container::flat_map<std::string, std::string>
                            assetInventory;
                        const std::array<const char*, 4> assetKeys = {
                            "PartNumber", "SerialNumber", "Manufacturer",
                            "Model"};
                        for (const auto& [key, value] : values)
                        {
                            if (std::find(assetKeys.begin(), assetKeys.end(),
                                          key) == assetKeys.end())
                            {
                                continue;
                            }
                            assetInventory[key] = std::get<std::string>(value);
                        }

                        // assume its a M.2 or something without a hsbp
                        if (parent == nullptr)
                        {
                            auto& drive = ownerlessDrives.emplace_back(
                                getDriveCount() + 1, true, true, true, false);
                            drive.createAsset(assetInventory);
                            return;
                        }

                        driveIndex += muxIndex;

                        if (parent->drives.size() <= driveIndex)
                        {
                            std::cerr << "Illegal drive index at " << path
                                      << " " << driveIndex << "\n";
                            return;
                        }
                        auto it = parent->drives.begin();
                        std::advance(it, driveIndex);

                        it->createAsset(assetInventory);
                    },
                    owner, path, "org.freedesktop.DBus.Properties", "GetAll",
                    "" /*all interface items*/);
            }
        },
        mapper::busName, mapper::path, mapper::interface, mapper::subtree, "/",
        0, std::array<const char*, 1>{nvmeType});
}

void populateMuxes(std::shared_ptr<boost::container::flat_set<Mux>> muxes,
                   std::string& rootPath)
{
    const static std::array<const std::string, 4> muxTypes = {
        "xyz.openbmc_project.Configuration.PCA9543Mux",
        "xyz.openbmc_project.Configuration.PCA9544Mux",
        "xyz.openbmc_project.Configuration.PCA9545Mux",
        "xyz.openbmc_project.Configuration.PCA9546Mux"};
    conn->async_method_call(
        [muxes](const boost::system::error_code ec,
                const GetSubTreeType& subtree) {
            if (ec)
            {
                std::cerr << "Error contacting mapper " << ec.message() << "\n";
                return;
            }
            std::shared_ptr<std::function<void()>> callback =
                std::make_shared<std::function<void()>>(
                    []() { updateAssets(); });
            size_t index = 0; // as we use a flat map, these are sorted
            for (const auto& [path, objDict] : subtree)
            {
                if (objDict.empty() || objDict.begin()->second.empty())
                {
                    continue;
                }

                const std::string& owner = objDict.begin()->first;
                const std::vector<std::string>& interfaces =
                    objDict.begin()->second;

                const std::string* interface = nullptr;
                for (const std::string& iface : interfaces)
                {
                    if (std::find(muxTypes.begin(), muxTypes.end(), iface) !=
                        muxTypes.end())
                    {
                        interface = &iface;
                        break;
                    }
                }
                if (interface == nullptr)
                {
                    std::cerr << "Cannot get mux type\n";
                    continue;
                }

                conn->async_method_call(
                    [path, muxes, callback, index](
                        const boost::system::error_code ec2,
                        const boost::container::flat_map<
                            std::string,
                            std::variant<uint64_t, std::vector<std::string>>>&
                            values) {
                        if (ec2)
                        {
                            std::cerr << "Error Getting Config "
                                      << ec2.message() << " " << __FUNCTION__
                                      << "\n";
                            return;
                        }
                        auto findBus = values.find("Bus");
                        auto findAddress = values.find("Address");
                        auto findChannelNames = values.find("ChannelNames");
                        if (findBus == values.end() ||
                            findAddress == values.end())
                        {
                            std::cerr << "Illegal configuration at " << path
                                      << "\n";
                            return;
                        }
                        size_t bus = static_cast<size_t>(
                            std::get<uint64_t>(findBus->second));
                        size_t address = static_cast<size_t>(
                            std::get<uint64_t>(findAddress->second));
                        std::vector<std::string> channels =
                            std::get<std::vector<std::string>>(
                                findChannelNames->second);
                        muxes->emplace(bus, address, channels.size(), index);
                        if (callback.use_count() == 1)
                        {
                            (*callback)();
                        }
                    },
                    owner, path, "org.freedesktop.DBus.Properties", "GetAll",
                    *interface);
                index++;
            }
        },
        mapper::busName, mapper::path, mapper::interface, mapper::subtree,
        rootPath, 1, muxTypes);
}

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
                    [path, owner](const boost::system::error_code ec2,
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
                        std::string parentPath =
                            std::filesystem::path(path).parent_path();
                        const auto& [backplane, status] = backplanes.emplace(
                            *name, std::make_shared<Backplane>(
                                       *bus, *address, *backplaneIndex, *name));
                        backplane->second->run(parentPath, owner);
                        populateMuxes(backplane->second->muxes, parentPath);
                    },
                    owner, path, "org.freedesktop.DBus.Properties", "GetAll",
                    configType);
            }
        },
        mapper::busName, mapper::path, mapper::interface, mapper::subtree, "/",
        0, std::array<const char*, 1>{configType});
}

static bool hsbpRequestAlertGpioEvents(
    const std::string& name, const std::function<void()>& handler,
    gpiod::line& gpioLine,
    boost::asio::posix::stream_descriptor& gpioEventDescriptor)
{
    // Find the GPIO line
    gpioLine = gpiod::find_line(name);
    if (!gpioLine)
    {
        std::cerr << "Failed to find the " << name << " line\n";
        return false;
    }

    try
    {
        gpioLine.request(
            {"hsbp-manager", gpiod::line_request::EVENT_BOTH_EDGES, 0});
    }
    catch (std::exception&)
    {
        std::cerr << "Failed to request events for " << name << "\n";
        return false;
    }

    int gpioLineFd = gpioLine.event_get_fd();
    if (gpioLineFd < 0)
    {
        std::cerr << "Failed to get " << name << " fd\n";
        return false;
    }

    gpioEventDescriptor.assign(gpioLineFd);

    gpioEventDescriptor.async_wait(
        boost::asio::posix::stream_descriptor::wait_read,
        [&name, handler](const boost::system::error_code ec) {
            if (ec)
            {
                std::cerr << name << " fd handler error: " << ec.message()
                          << "\n";
                return;
            }
            handler();
        });
    return true;
}

/******************************************************************************************
 *   HSBP Position           CPLD SMB Address
 *        1                   0xD0(0x68 7 bit)
 *        2                   0xD2(0x69 7 bit)
 *  we have max 2 HSBP per system. Closed chassis systems will either have 0 or
 *  2 HSBP's.
 *******************************************************************************************/
static constexpr uint8_t hsbpI2cBus = 4;
static constexpr uint8_t allDrivesWithStatusBit = 17;
static constexpr uint8_t statusAllDrives = (allDrivesWithStatusBit - 1);
static constexpr uint8_t allClockBitsDb2000 = 25;
static constexpr uint8_t statusAllClocksDb2000 = (allClockBitsDb2000 - 1);
static constexpr uint8_t singleDriveWithStatusBit = 9;
static constexpr uint8_t statusSingleDrive = (singleDriveWithStatusBit - 1);
static constexpr uint8_t maxDrivesPerHsbp = 8;

static std::bitset<allDrivesWithStatusBit> drivePresenceStatus;
static std::bitset<allClockBitsDb2000> driveClockStatus;
static constexpr uint8_t hsbpCpldSmbaddr1 = 0x68;
static constexpr uint8_t hsbpCpldSmbaddr2 = 0x69;
static constexpr uint8_t hsbpCpldReg8 = 0x8;
static constexpr uint8_t hsbpCpldReg9 = 0x9;
static constexpr uint8_t db2000SlaveAddr = 0x6d;
static constexpr uint8_t db2000RegByte0 = 0x80;
static constexpr uint8_t db2000RegByte1 = 0x81;
static constexpr uint8_t db2000RegByte2 = 0x82;
static int hsbpFd;

/********************************************************************
 *  DB2000 Programming guide for PCIe Clocks enable/disable
 *  CPU 0
 * =================================================================
 *  slot        Byte        bit number
 *  Position   position
 * =================================================================
 *  7            0           5
 *  6            0           4
 *  5            0           3
 *  4            2           7
 *  3            1           3
 *  2            1           2
 *  1            1           1
 *  0            1           0
 *
 *  CPU 1
 * =================================================================
 *  slot        Byte        bit number
 *  Position   position
 * =================================================================
 *  7            1           6
 *  6            1           7
 *  5            2           0
 *  4            2           1
 *  3            1           5
 *  2            2           4
 *  1            2           2
 *  0            2           3
 *********************************************************************/
std::optional<int>
    updateClocksStatus(std::bitset<allDrivesWithStatusBit> nvmeDriveStatus)
{
    std::bitset<allClockBitsDb2000> nvmeClockStatus;
    /* mapping table for nvme drive index(0-15) to DB2000 register bit fields */
    constexpr std::array<int, statusAllDrives> slotToClockTable = {
        8, 9, 10, 11, 23, 3, 4, 5, 19, 18, 20, 13, 17, 16, 15, 14};

    /* scan through all drives(except the status bit) and update corresponding
     * clock bit */
    for (std::size_t i = 0; i < (nvmeDriveStatus.size() - 1); i++)
    {
        if (nvmeDriveStatus.test(i))
        {
            nvmeClockStatus.set(slotToClockTable[i]);
        }
        else
        {
            nvmeClockStatus.reset(slotToClockTable[i]);
        }
    }

    if (ioctl(hsbpFd, I2C_SLAVE_FORCE, db2000SlaveAddr) < 0)
    {
        std::cerr << "unable to set DB2000 address to " << db2000SlaveAddr
                  << "\n";
        return std::nullopt;
    }
    int ret = i2c_smbus_write_byte_data(
        hsbpFd, db2000RegByte0,
        static_cast<unsigned char>(nvmeClockStatus.to_ulong()));

    if (ret < 0)
    {
        std::cerr << "Error: unable to write data to clock register "
                  << __FUNCTION__ << __LINE__ << "\n";
        return ret;
    }

    ret = i2c_smbus_write_byte_data(
        hsbpFd, db2000RegByte1,
        static_cast<unsigned char>((nvmeClockStatus >> 8).to_ulong()));

    if (ret < 0)
    {
        std::cerr << "Error: unable to write data to clock register "
                  << __FUNCTION__ << __LINE__ << "\n";
        return ret;
    }

    ret = i2c_smbus_write_byte_data(
        hsbpFd, db2000RegByte2,
        static_cast<unsigned char>((nvmeClockStatus >> 16).to_ulong()));

    if (ret < 0)
    {
        std::cerr << "Error: unable to write data to clock register "
                  << __FUNCTION__ << __LINE__ << "\n";
        return ret;
    }
    // Update global clock status
    driveClockStatus = nvmeClockStatus;
    driveClockStatus.set(statusAllClocksDb2000, 1);
    return 0;
}

std::bitset<singleDriveWithStatusBit>
    getSingleHsbpDriveStatus(const uint8_t cpldSmbaddr)
{
    std::bitset<singleDriveWithStatusBit> singleDriveStatus;

    // probe
    if (ioctl(hsbpFd, I2C_SLAVE_FORCE, cpldSmbaddr) < 0)
    {
        std::cerr << "Failed to talk to cpldSmbaddr :  " << cpldSmbaddr << "\n";
        return singleDriveStatus;
    }

    // read status of lower four drive connectivity
    int valueReg8 = i2c_smbus_read_byte_data(hsbpFd, hsbpCpldReg8);
    if (valueReg8 < 0)
    {
        std::cerr << "Error: Unable to read cpld reg 0x8 " << __FUNCTION__
                  << __LINE__ << "\n";
        return singleDriveStatus;
    }

    // read status of upper four drive connectivity
    int valueReg9 = i2c_smbus_read_byte_data(hsbpFd, hsbpCpldReg9);
    if (valueReg9 < 0)
    {
        std::cerr << "Error: Unable to read cpld reg 0x9 " << __FUNCTION__
                  << __LINE__ << "\n";
        return singleDriveStatus;
    }

    // Find drives which have NVMe drive connected
    for (int loop = 0; loop < (singleDriveWithStatusBit - 1); loop++)
    {
        // Check if NVME drive detected(corresponding bit numbers of reg8 and
        // reg9 are 1 and 0 resp)
        if (valueReg8 & (1U << loop))
        {
            if ((valueReg9 & (1U << loop)) == 0)
            {
                singleDriveStatus.set(loop, 1);
            }
        }
    }

    // Reading successful, set the statusok bit
    singleDriveStatus.set(statusSingleDrive, 1);
    return singleDriveStatus;
}

/* Try reading both HSBP and report back if atleast one of them is found to be
   connected. Status bit is set by the function even if one HSBP is responding
 */
std::bitset<allDrivesWithStatusBit> getCompleteDriveStatus(void)
{
    std::bitset<singleDriveWithStatusBit> singleDrvStatus;
    std::bitset<allDrivesWithStatusBit> currDriveStatus;

    singleDrvStatus = getSingleHsbpDriveStatus(hsbpCpldSmbaddr1);

    if (singleDrvStatus[statusSingleDrive] == 1)
    {
        for (int i = 0; i < maxDrivesPerHsbp; i++)
        {
            currDriveStatus[i] = singleDrvStatus[i];
        }
        // set valid bit if a single hsbp drive status is valid
        currDriveStatus.set(statusAllDrives);
    }
    else
    {
        currDriveStatus &= (~0xFF);
    }

    singleDrvStatus = getSingleHsbpDriveStatus(hsbpCpldSmbaddr2);
    if (singleDrvStatus[statusSingleDrive] == 1)
    {
        for (int i = maxDrivesPerHsbp, j = 0; i < (allDrivesWithStatusBit - 1);
             i++, j++)
        {
            currDriveStatus[i] = singleDrvStatus[j];
        }
        // set valid bit if a single hsbp drive status is valid
        currDriveStatus.set(statusAllDrives);
    }
    else
    {
        currDriveStatus &= (~(0xFF << maxDrivesPerHsbp));
    }
    return currDriveStatus;
}

void cpldReadingInit(void)
{
    hsbpFd = open(("/dev/i2c-" + std::to_string(hsbpI2cBus)).c_str(),
                  O_RDWR | O_CLOEXEC);
    if (hsbpFd < 0)
    {
        std::cerr << "unable to open hsbpI2cBus " << hsbpI2cBus << "\n";
        return;
    }

    std::bitset<allDrivesWithStatusBit> currDrvStatus =
        getCompleteDriveStatus();
    if (currDrvStatus[statusAllDrives] == 1)
    {
        // update global drive presence for next time comparison
        drivePresenceStatus = currDrvStatus;
        std::optional<int> updateStatus =
            updateClocksStatus(drivePresenceStatus);
        if (updateStatus.has_value())
        {
            if (updateStatus == -1)
            {
                std::cerr << "error: DB2000 register read issue "
                          << "\n";
                close(hsbpFd);
                hsbpFd = -1;
            }
        }
        else
        {
            std::cerr << "error: DB2000 i2c access issue "
                      << "\n";
            close(hsbpFd);
            hsbpFd = -1;
        }
    }
    else
    {
        close(hsbpFd);
        hsbpFd = -1;
    }
}

// Callback handler passed to hsbpRequestAlertGpioEvents:
static void nvmeLvc3AlertHandler()
{
    if (hsbpFd >= 0)
    {
        gpiod::line_event gpioLineEvent = nvmeLvc3AlertLine.event_read();

        if (gpioLineEvent.event_type == gpiod::line_event::FALLING_EDGE)
        {
            /* Step 1: Either drive is removed or inserted; read the CPLD reg 8
               and 9 to determine if drive is added or removed. Need to compare
                    current number of drives with previous state to determine
               it.
            */
            std::bitset<allDrivesWithStatusBit> currDrvStat =
                getCompleteDriveStatus();
            if (currDrvStat[statusAllDrives] == 1)
            {
                if (drivePresenceStatus != currDrvStat)
                {
                    uint32_t tmpVar = static_cast<uint32_t>(
                        (drivePresenceStatus ^ currDrvStat).to_ulong());
                    uint32_t indexDrive = 0;
                    while (tmpVar > 0)
                    {
                        if (tmpVar & 1)
                        {
                            if (drivePresenceStatus[indexDrive] == 0)
                            {
                                logDeviceAdded(
                                    "Drive", std::to_string(indexDrive), "N/A");
                            }
                            else
                            {
                                logDeviceRemoved(
                                    "Drive", std::to_string(indexDrive), "N/A");
                            }
                        }
                        indexDrive++;
                        tmpVar >>= 1;
                    }
                    // update global drive presence for next time comparison
                    drivePresenceStatus = currDrvStat;

                    // Step 2: disable or enable the pcie clock for
                    // corresponding drive
                    std::optional<int> tmpUpdStatus =
                        updateClocksStatus(currDrvStat);
                    if (tmpUpdStatus.has_value())
                    {
                        if (tmpUpdStatus == -1)
                        {
                            std::cerr << "error: DB2000 register read issue "
                                      << "\n";
                            close(hsbpFd);
                            hsbpFd = -1;
                        }
                    }
                    else
                    {
                        std::cerr << "error: DB2000 i2c access issue "
                                  << "\n";
                        close(hsbpFd);
                        hsbpFd = -1;
                    }
                }
                // false alarm
                else
                {
                    std::cerr
                        << "False alarm detected by HSBP; no action taken \n";
                }
            }
            else
            {
                close(hsbpFd);
                hsbpFd = -1;
            }
        }

        nvmeLvc3AlertEvent.async_wait(
            boost::asio::posix::stream_descriptor::wait_read,
            [](const boost::system::error_code ec) {
                if (ec)
                {
                    std::cerr << "nvmealert handler error: " << ec.message()
                              << "\n";
                    return;
                }
                nvmeLvc3AlertHandler();
            });
    }
}

int main()
{
    boost::asio::steady_timer callbackTimer(io);

    conn->request_name(busName);

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

    sdbusplus::bus::match::match drive(
        *conn,
        "type='signal',member='PropertiesChanged',arg0='xyz.openbmc_project."
        "Inventory.Item.NVMe'",
        [&callbackTimer](sdbusplus::message::message& message) {
            callbackTimer.expires_after(std::chrono::seconds(2));
            if (message.get_sender() == conn->get_unique_name())
            {
                return;
            }
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
                updateAssets();
            });
        });

    cpldReadingInit();

    if (hsbpFd >= 0)
    {
        if (!hsbpRequestAlertGpioEvents("FM_SMB_BMC_NVME_LVC3_ALERT_N",
                                        nvmeLvc3AlertHandler, nvmeLvc3AlertLine,
                                        nvmeLvc3AlertEvent))
        {
            std::cerr << "error: Unable to monitor events on HSBP Alert line "
                      << "\n";
        }
    }

    auto iface =
        objServer.add_interface("/xyz/openbmc_project/inventory/item/storage",
                                "xyz.openbmc_project.inventory.item.storage");

    io.post([]() { populate(); });
    setupPowerMatch(conn);
    io.run();
    close(hsbpFd);
    hsbpFd = -1;
}
