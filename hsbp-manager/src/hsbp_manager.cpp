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
static gpiod::line Nvme_Lvc3_Alert_Line;
static boost::asio::posix::stream_descriptor Nvme_Lvc3_Alert_Event(io);

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

// Function to be called from main:

static bool HsbpRequestAlertGpioEvents(
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
 *2 HSBP's.
 *******************************************************************************************/
using std::bitset;
#define HSBP_I2CBUS 4
#define ALL_DRIVES_WITH_STATUS_BIT 17
#define ALL_CLOCK_BITS_DB2000 25
std::bitset<ALL_DRIVES_WITH_STATUS_BIT> drivePresenceStatus(0);
std::bitset<ALL_CLOCK_BITS_DB2000> driveClockStatus(0);
constexpr const uint8_t HsbpCpldSmbaddr1 = 0x68;
constexpr const uint8_t HsbpCpldSmbaddr2 = 0x69;
constexpr const uint8_t Db2000SlaveAddr = 0x6d;
int file = -1;

// DB2000 Programming guide for PCIe Clocks enable/disable
// CPU 0
//=================================================================
// slot        Byte        bit number
// Position   position
//=================================================================
// 7            0           5
// 6            0           4
// 5            0           3
// 4            2           7
// 3            1           3
// 2            1           2
// 1            1           1
// 0            1           0

// CPU 1
//=================================================================
// slot        Byte        bit number
// Position   position
//=================================================================
// 7            1           6
// 6            1           7
// 5            2           0
// 4            2           1
// 3            1           5
// 2            2           4
// 1            2           2
// 0            2           3

bitset<25> readcurrent_clocks_status(void)
{
    int valuebyte0, valuebyte1, valuebyte2;
    bitset<25> tmpclockstatus(0);
    constexpr uint8_t byte0 = 0x80;
    constexpr uint8_t byte1 = 0x81;
    constexpr uint8_t byte2 = 0x82;

    if (ioctl(file, I2C_SLAVE_FORCE, Db2000SlaveAddr) < 0)
    {
        std::cerr << "unable to set DB2000 address to " << Db2000SlaveAddr
                  << "\n";
        return tmpclockstatus;
    }

    valuebyte0 = i2c_smbus_read_byte_data(file, byte0);
    if (valuebyte0 < 0)
    {
        std::cerr << "Error: reading byte0 of DB2000"
                  << "\n";
        return tmpclockstatus;
    }
    tmpclockstatus |= valuebyte0;

    valuebyte1 = i2c_smbus_read_byte_data(file, byte1);
    if (valuebyte1 < 0)
    {
        std::cerr << "Error: reading byte1 of DB2000"
                  << "\n";
        return tmpclockstatus;
    }
    tmpclockstatus |= (valuebyte1 < 8);

    valuebyte2 = i2c_smbus_read_byte_data(file, byte2);
    if (valuebyte2 < 0)
    {
        std::cerr << "Error: reading byte2 of DB2000"
                  << "\n";
        return tmpclockstatus;
    }
    tmpclockstatus |= (valuebyte2 < 16);

    tmpclockstatus.set(24, 1);

    return tmpclockstatus;
}

bool update_clocks_status(bitset<17> nvmedriveStatus,
                          bitset<25> nvmeclockstatus)
{
    int ret = -1;
    /* mapping table for nvme drive index(0-15) to DB2000 register bit fields */
    int slotToClockTable[] = {8,  9,  10, 11, 23, 3,  4,  5,
                              19, 18, 20, 13, 17, 16, 15, 14};

    for (std::size_t i = 0; i < (nvmedriveStatus.size() - 1); i++)
    {
        if (nvmedriveStatus.test(i))
        {
            nvmeclockstatus.set(slotToClockTable[i]);
            driveClockStatus.set(i);
        }
        else
        {
            nvmeclockstatus.reset(slotToClockTable[i]);
            driveClockStatus.reset(i);
        }
    }

    ret = i2c_smbus_write_byte_data(
        file, 0x80, static_cast<unsigned char>(nvmeclockstatus.to_ulong()));

    if (ret < 0)
    {
        std::cerr << "Error: unable to write data to clock register "
                  << __FUNCTION__ << __LINE__ << "\n";
        return false;
    }

    ret = i2c_smbus_write_byte_data(
        file, 0x81,
        static_cast<unsigned char>((nvmeclockstatus >> 8).to_ulong()));

    if (ret < 0)
    {
        std::cerr << "Error: unable to write data to clock register "
                  << __FUNCTION__ << __LINE__ << "\n";
        return false;
    }

    ret = i2c_smbus_write_byte_data(
        file, 0x82,
        static_cast<unsigned char>((nvmeclockstatus >> 16).to_ulong()));

    if (ret < 0)
    {
        std::cerr << "Error: unable to write data to clock register "
                  << __FUNCTION__ << __LINE__ << "\n";
        return false;
    }
    return true;
}

bitset<9> getSingleHsbpDriveStatus(const uint8_t cpld_smbaddr)
{
    int valuereg8, valuereg9;
    bitset<9> singledriveStatus(0);

    // probe
    if (ioctl(file, I2C_SLAVE_FORCE, cpld_smbaddr) < 0)
    {
        std::cerr << "Failed to talk to cpld_smbaddr :  " << cpld_smbaddr
                  << "\n";
        return singledriveStatus;
    }

    // read status of lower four drive connectivity
    valuereg8 = i2c_smbus_read_byte_data(file, 0x8);
    if (valuereg8 < 0)
    {
        std::cerr << "Error: Unable to read cpld reg 0x8 " << __FUNCTION__
                  << __LINE__ << "\n";
        return singledriveStatus;
    }

    // read status of upper four drive connectivity
    valuereg9 = i2c_smbus_read_byte_data(file, 0x9);
    if (valuereg9 < 0)
    {
        std::cerr << "Error: Unable to read cpld reg 0x9 " << __FUNCTION__
                  << __LINE__ << "\n";
        return singledriveStatus;
    }

    int loop;
    // Find drives which have NVMe drive connected
    for (loop = 0; loop < 8; loop++)
    {
        if ((valuereg8 & (1U << loop)) &&
            (valuereg9 & ((1U << loop))) == 0) // NVME drive detected
        {
            singledriveStatus.set(loop, 1);
        }
    }

    // Reading successful, set the statusok bit
    singledriveStatus.set(8, 1);
    return singledriveStatus;
}

// Try reading both HSBP and report back if atleast one of them is found to be
// connected. Status bit is set by the function even if one HSBP is responding
bitset<17> getCompleteDriveStatus(void)
{
    bitset<9> singledriveStatus(0);
    bitset<17> currentdriveStatus(0);
    int i, j;

    singledriveStatus = getSingleHsbpDriveStatus(HsbpCpldSmbaddr1);

    if (singledriveStatus[8] == 1)
    {
        for (i = 0; i < 8; i++)
        {
            currentdriveStatus[i] = singledriveStatus[i];
        }
        // set valid bit if a single hsbp drive status is valid
        currentdriveStatus.set(16);
    }
    else
    {
        currentdriveStatus &= (~0xFF);
    }

    singledriveStatus = getSingleHsbpDriveStatus(HsbpCpldSmbaddr2);
    if (singledriveStatus[8] == 1)
    {
        for (i = 8, j = 0; i < 16; i++, j++)
        {
            currentdriveStatus[i] = singledriveStatus[j];
        }
        // set valid bit if a single hsbp drive status is valid
        currentdriveStatus.set(16);
    }
    else
    {
        currentdriveStatus &= (~(0xFF << 8));
    }
    return currentdriveStatus;
}

void cpld_reading_init(void)
{
    bitset<9> singledriveStatus1(0), singledriveStatus2(0);
    bitset<25> clockstatus(0);
    file = open(("/dev/i2c-" + std::to_string(HSBP_I2CBUS)).c_str(),
                O_RDWR | O_CLOEXEC);
    if (file < 0)
    {
        std::cerr << "unable to open HSBP_I2CBUS " << HSBP_I2CBUS << "\n";
        return;
    }
    bitset<17> currentdrivestatus = getCompleteDriveStatus();
    if (currentdrivestatus[16] == 1)
    {
        // update global drive presence for next time comparison
        drivePresenceStatus = currentdrivestatus;
    }
    else
    {
        close(file);
        return;
    }

    driveClockStatus = readcurrent_clocks_status();
    if (driveClockStatus[24] == 0)
    {
        std::cerr << "error: unable to detect any devices "
                  << "\n";
        close(file);
        return;
    }

    if (!update_clocks_status(drivePresenceStatus, driveClockStatus))
    {
        std::cerr << "error: unable to update clocks "
                  << "\n";
        close(file);
        return;
    }
    // Set the valid bit
    driveClockStatus.set(24, 1);
}

// Callback handler passed to HsbpRequestAlertGpioEvents:
static void NVME_LVC3_ALERT_Handler()
{
    gpiod::line_event gpioLineEvent = Nvme_Lvc3_Alert_Line.event_read();

    bool nvmealert =
        gpioLineEvent.event_type == gpiod::line_event::FALLING_EDGE;
    if (nvmealert)
    {
        // Step 1: Either drive is removed or inserted; read the CPLD reg 8 and
        //         9 to determine
        //         if drive is added or removed. Need to compare current number
        //         of drives with previous state to determine it.
        bitset<17> currentdrivestatus = getCompleteDriveStatus();
        std::cerr << "On new alert value of  drivePresenceStatus and "
                     "currentdrivestatus is : "
                  << drivePresenceStatus << "\t" << currentdrivestatus << "\n";
        if (currentdrivestatus[16] == 1)
        {
            if (drivePresenceStatus == currentdrivestatus) // false alarm
            {
                std::cerr << "False alarm detected by HSBP; no action taken \n";
            }
            unsigned int tmpvar = static_cast<int>(
                (drivePresenceStatus ^ currentdrivestatus).to_ulong());
            unsigned int indexDrive = 0;
            while (tmpvar >>= 1)
            {
                indexDrive++;
            }
            if (drivePresenceStatus[indexDrive] == 0)
            {
                logDeviceAdded("Drive", std::to_string(indexDrive), "N/A");
            }
            else
            {
                logDeviceRemoved("Drive", std::to_string(indexDrive), "N/A");
            }

            // update global drive presence for next time comparison
            drivePresenceStatus = currentdrivestatus;
        }

        // Step 2: disable or enable the pcie clock for corresponding drive
        driveClockStatus = readcurrent_clocks_status();
        if (driveClockStatus[24] == 0)
        {
            std::cerr
                << "FATAL error in reading the drive clock status; disabling "
                   "all clocks \n";
            update_clocks_status(currentdrivestatus.reset(),
                                 driveClockStatus.reset());
            return;
        }
        if (update_clocks_status(currentdrivestatus, driveClockStatus))
        {
            // Update the pcie clocks
            driveClockStatus.set(24, 1);
        }
        else
        {
            std::cerr
                << "error encountered in updating the pcie clock status \n";
        }
    }
    Nvme_Lvc3_Alert_Event.async_wait(
        boost::asio::posix::stream_descriptor::wait_read,
        [](const boost::system::error_code ec) {
            if (ec)
            {
                std::cerr << "nvmealert handler error: " << ec.message()
                          << "\n";
                return;
            }
            NVME_LVC3_ALERT_Handler();
        });
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

    cpld_reading_init();

    if (!HsbpRequestAlertGpioEvents(
            "FM_SMB_BMC_NVME_LVC3_ALERT_N", NVME_LVC3_ALERT_Handler,
            Nvme_Lvc3_Alert_Line, Nvme_Lvc3_Alert_Event))
    {
        return -1;
    }

    auto iface =
        objServer.add_interface("/xyz/openbmc_project/inventory/item/storage",
                                "xyz.openbmc_project.inventory.item.storage");

    io.post([]() { populate(); });
    setupPowerMatch(conn);
    io.run();
    close(file);
}
