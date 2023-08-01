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

#include <algorithm>
#include <bitset>
#include <boost/algorithm/string/replace.hpp>
#include <boost/asio/posix/stream_descriptor.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/container/flat_set.hpp>
#include <filesystem>
#include <forward_list>
#include <fstream>
#include <gpiod.hpp>
#include <iostream>
#include <list>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/bus/match.hpp>
#include <string>
#include <utility>

extern "C" {
#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
}

/****************************************************************************/
/******************** Global Constants/Type Declarations ********************/
/****************************************************************************/
constexpr const char* hsbpCpldInft =
    "xyz.openbmc_project.Configuration.Intel_HSBP_CPLD";
constexpr const char* hsbpConfigIntf =
    "xyz.openbmc_project.Configuration.HSBPConfiguration";
constexpr const char* nvmeIntf = "xyz.openbmc_project.Inventory.Item.NVMe";
constexpr const char* busName = "xyz.openbmc_project.HsbpManager";

constexpr size_t scanRateSeconds = 5;
constexpr size_t maxDrives = 8; // only 1 byte alloted

using NvmeMapping = std::vector<std::string>;
/***************************** End of Section *******************************/

/****************************************************************************/
/**************************** Enums Definitions *****************************/
/****************************************************************************/
enum class AppState : uint8_t
{
    idle,
    loadingHsbpConfig,
    hsbpConfigLoaded,
    loadingComponents,
    componentsLoaded,
    loadingBackplanes,
    backplanesLoaded,
    loadingDrives,
    drivesLoaded
};

enum class BlinkPattern : uint8_t
{
    off = 0x0,
    error = 0x2,
    terminate = 0x3
};
/***************************** End of Section *******************************/

/****************************************************************************/
/************ HSBP Configuration related struct/class Definitions ***********/
/****************************************************************************/
struct HsbpConfig
{
    size_t rootBus;
    std::vector<std::string> supportedHsbps;
    std::unordered_map<std::string, NvmeMapping> hsbpNvmeMap;
    std::vector<std::string> clockBufferTypes;
    std::vector<std::string> ioExpanderTypes;

    void clearConfig()
    {
        rootBus = -1;
        supportedHsbps.clear();
        hsbpNvmeMap.clear();
        clockBufferTypes.clear();
        ioExpanderTypes.clear();
    }
};

class ClockBuffer
{
    size_t bus;
    size_t address;
    std::string modeOfOperation;
    size_t outCtrlBaseAddr;
    size_t outCtrlByteCount;
    std::unordered_map<std::string, std::vector<std::string>> byteMap;
    std::string name;
    std::string type;

    int file = -1;
    bool initialized = false;

    void initialize()
    {
        /* Execute below operation only when mode of operation is SMBus. If
         * mode of operation is IO, then the IO value will determine the
         * disable/enable of clock output. Open the Clock buffer device */
        if (modeOfOperation == "SMBus")
        {
            if (file < 0)
            {
                file = open(("/dev/i2c-" + std::to_string(bus)).c_str(),
                            O_RDWR | O_CLOEXEC);
                if (file < 0)
                {
                    std::cerr << "ClockBuffer : \"" << name
                              << "\" - Unable to open bus : " << bus << "\n";
                    return;
                }
            }

            if (ioctl(file, I2C_SLAVE_FORCE, address) < 0)
            {
                std::cerr << "ClockBuffer : \"" << name
                          << "\" - Unable to set address to " << address
                          << "\n";
                return;
            }
        }

        initialized = true;
        std::cerr << "ClockBuffer : \"" << name << "\" initialized\n";
    }

  public:
    ClockBuffer(
        size_t busIn, size_t addressIn, std::string& modeOfOperationIn,
        size_t outCtrlBaseAddrIn, size_t outCtrlByteCountIn,
        std::unordered_map<std::string, std::vector<std::string>>& byteMapIn,
        std::string& nameIn, std::string& typeIn) :
        bus(busIn),
        address(addressIn), modeOfOperation(std::move(modeOfOperationIn)),
        outCtrlBaseAddr(outCtrlBaseAddrIn),
        outCtrlByteCount(outCtrlByteCountIn), byteMap(std::move(byteMapIn)),
        name(std::move(nameIn)), type(std::move(typeIn))
    {
        initialize();
    }

    bool isInitialized()
    {
        if (!initialized)
        {
            /* There was an issue with the initialization of this component. Try
             * to invoke initialization again */
            initialize();
        }
        return initialized;
    }

    bool disableClocks()
    {
        /* Execute below operation only when mode of operation is SMBus. By
         * default the clock buffer is configured to follow OE pin output, so we
         * need to set the output value to 0 to disable the clock outputs. If
         * mode of operation is IO, then the IO value will determine the
         * disable/enable of clock output */

        if (modeOfOperation != "SMBus")
        {
            /* The clock is enabled using IO expander. No action needed from
             * here */
            return true;
        }

        for (uint8_t i = 0; i < outCtrlByteCount; i++)
        {
            std::string byteName = "Byte" + std::to_string(i);

            auto byte = byteMap.find(byteName);
            if (byte == byteMap.end())
            {
                std::cerr << "ClockBuffer : \"" << name
                          << "\" - Byte map error ! Unable to find " << byteName
                          << "\n";
                return false;
            }

            /* Get current value of output control register */
            int read = i2c_smbus_read_byte_data(
                file, static_cast<uint8_t>(outCtrlBaseAddr + i));
            if (read < 0)
            {
                std::cerr << "ClockBuffer : \"" << name
                          << "\" - Error: Unable to read data from clock "
                             "buffer register\n";
                return false;
            }

            std::bitset<8> currByte(read);
            bool writeRequired = false;

            /* Set zero only at bit position that we have a NVMe drive (i.e.
             * ignore where byteMap is "-"). We do not want to touch other
             * bits */
            for (uint8_t bit = 0; bit < 8; bit++)
            {
                if (byte->second.at(bit) != "-")
                {
                    writeRequired = true;
                    currByte.reset(bit);
                }
            }

            if (writeRequired)
            {
                int ret = i2c_smbus_write_byte_data(
                    file, static_cast<uint8_t>(outCtrlBaseAddr + i),
                    static_cast<uint8_t>(currByte.to_ulong()));

                if (ret < 0)
                {
                    std::cerr << "ClockBuffer : \"" << name
                              << "\" - Error: Unable to write data to clock "
                                 "buffer register\n";
                    return false;
                }
            }
        }

        return true;
    }

    std::string getName()
    {
        return name;
    }

    bool enableDisableClock(std::forward_list<std::string>& nvmeDrivesInserted,
                            std::forward_list<std::string>& nvmeDrivesRemoved)
    {
        if (modeOfOperation != "SMBus")
        {
            /* The clock is enabled using IO expander. No action needed from
             * here */
            return true;
        }

        if (nvmeDrivesInserted.empty() && nvmeDrivesRemoved.empty())
        {
            /* There are no drives to update */
            return true;
        }

        for (uint8_t i = 0; i < outCtrlByteCount; i++)
        {
            std::string byteName = "Byte" + std::to_string(i);

            auto byte = byteMap.find(byteName);
            if (byte == byteMap.end())
            {
                std::cerr << "ClockBuffer : \"" << name
                          << "\" - Byte map error ! Unable to find " << byteName
                          << "\n";
                return false;
            }

            /* Get current value of output control register */
            int read = i2c_smbus_read_byte_data(
                file, static_cast<uint8_t>(outCtrlBaseAddr + i));
            if (read < 0)
            {
                std::cerr << "ClockBuffer : \"" << name
                          << "\" - Error: Unable to read data from clock "
                             "buffer register\n";
                return false;
            }

            std::bitset<8> currByte(read);
            bool writeRequired = false;

            /* Set the bit if the NVMe drive is found in nvmeDrivesInserted, and
             * reset the bit if found in nvmeDrivesRemoved */
            for (uint8_t bit = 0; bit < 8; bit++)
            {
                /* The remove function returns number of elements removed from
                 * list indicating the presence of the drive and also removing
                 * it form the list */
                if (nvmeDrivesInserted.remove(byte->second.at(bit)))
                {
                    writeRequired = true;
                    currByte.set(bit);
                    continue;
                }

                if (nvmeDrivesRemoved.remove(byte->second.at(bit)))
                {
                    writeRequired = true;
                    currByte.reset(bit);
                }
            }

            if (!writeRequired)
            {
                /* No Write is required as there are no changes */
                continue;
            }

            int ret = i2c_smbus_write_byte_data(
                file, static_cast<uint8_t>(outCtrlBaseAddr + i),
                static_cast<uint8_t>(currByte.to_ulong()));
            if (ret < 0)
            {
                std::cerr << "ClockBuffer : \"" << name
                          << "\" - Error: Unable to write data to clock "
                             "buffer register\n";
                return false;
            }
        }

        return true;
    }

    ~ClockBuffer()
    {
        if (file >= 0)
        {
            close(file);
        }
    }
};

class IoExpander
{
    size_t bus;
    size_t address;
    size_t confIORegAddr;
    size_t outCtrlBaseAddr;
    size_t outCtrlByteCount;
    std::unordered_map<std::string, std::vector<std::string>> ioMap;
    std::string name;
    std::string type;

    int file = -1;
    bool initialized = false;

    void initialize()
    {
        /* Open IO controller device */
        if (file < 0)
        {
            file = open(("/dev/i2c-" + std::to_string(bus)).c_str(),
                        O_RDWR | O_CLOEXEC);
            if (file < 0)
            {
                std::cerr << "IoExpander : " << name
                          << " - Unable to open bus : " << bus << "\n";
                return;
            }
        }

        if (ioctl(file, I2C_SLAVE_FORCE, address) < 0)
        {
            std::cerr << "IoExpander : \"" << name
                      << "\" - Unable to set address to " << address << "\n";
            return;
        }

        initialized = true;
        std::cerr << "IoExpander : \"" << name << "\" initialized\n";
    }

  public:
    IoExpander(
        size_t busIn, size_t addressIn, size_t confIORegAddrIn,
        size_t outCtrlBaseAddrIn, size_t outCtrlByteCountIn,
        std::unordered_map<std::string, std::vector<std::string>>& ioMapIn,
        std::string& nameIn, std::string& typeIn) :
        bus(busIn),
        address(addressIn), confIORegAddr(confIORegAddrIn),
        outCtrlBaseAddr(outCtrlBaseAddrIn),
        outCtrlByteCount(outCtrlByteCountIn), ioMap(std::move(ioMapIn)),
        name(std::move(nameIn)), type(std::move(typeIn))
    {
        initialize();
    }

    bool isInitialized()
    {
        if (!initialized)
        {
            /* There was an issue with the initialization of this component. Try
             * to invoke initialization again */
            initialize();
        }
        return initialized;
    }

    bool disableIO()
    {
        /* Initialize the IO expander Control register to configure the IO ports
         * as outputs and set the output to low by default */
        for (uint8_t i = 0; i < outCtrlByteCount; i++)
        {
            std::string ioName = "IO" + std::to_string(i);

            auto io = ioMap.find(ioName);
            if (io == ioMap.end())
            {
                std::cerr << "IoExpander : \"" << name
                          << "\" - IO map error ! Unable to find " << ioName
                          << "\n";
                return false;
            }

            /* Get current value of IO configuration register */
            int read1 = i2c_smbus_read_byte_data(
                file, static_cast<uint8_t>(confIORegAddr + i));
            if (read1 < 0)
            {
                std::cerr << "IoExpander : \"" << name
                          << "\" - Error: Unable to read data from io expander "
                             "IO control register\n";
                return false;
            }

            /* Get current value of IO Ouput register */
            int read2 = i2c_smbus_read_byte_data(
                file, static_cast<uint8_t>(confIORegAddr + i));
            if (read2 < 0)
            {
                std::cerr << "IoExpander : \"" << name
                          << "\" - Error: Unable to read data from io expander "
                             "IO output register\n";
                return false;
            }

            bool writeRequired = false;
            std::bitset<8> currCtrlVal(read1);
            std::bitset<8> currOutVal(read2);

            /* Set zero only at bit position that we have a NVMe drive (i.e.
             * ignore where ioMap is "-"). We do not want to touch other
             * bits */
            for (uint8_t bit = 0; bit < 8; bit++)
            {
                if (io->second.at(bit) != "-")
                {
                    writeRequired = true;
                    currCtrlVal.reset(bit);
                    currOutVal.reset(bit);
                }
            }

            if (writeRequired)
            {
                int ret1 = i2c_smbus_write_byte_data(
                    file, static_cast<uint8_t>(confIORegAddr + i),
                    static_cast<uint8_t>(currCtrlVal.to_ulong()));
                if (ret1 < 0)
                {
                    std::cerr
                        << "IoExpander : \"" << name
                        << "\" - Error: Unable to write data to IO expander "
                           "IO control register\n";
                    return false;
                }

                int ret2 = i2c_smbus_write_byte_data(
                    file, static_cast<uint8_t>(outCtrlBaseAddr + i),
                    static_cast<uint8_t>(currOutVal.to_ulong()));
                if (ret2 < 0)
                {
                    std::cerr
                        << "IoExpander : \"" << name
                        << "\" - Error: Unable to write data to IO expander "
                           "IO output register\n";
                    return false;
                }
            }
        }

        return true;
    }

    std::string getName()
    {
        return name;
    }

    bool enableDisableOuput(std::forward_list<std::string>& nvmeDrivesInserted,
                            std::forward_list<std::string>& nvmeDrivesRemoved)
    {
        if (nvmeDrivesInserted.empty() && nvmeDrivesRemoved.empty())
        {
            /* There are no drives to update */
            return true;
        }

        for (uint8_t i = 0; i < outCtrlByteCount; i++)
        {
            std::string ioName = "IO" + std::to_string(i);

            auto io = ioMap.find(ioName);
            if (io == ioMap.end())
            {
                std::cerr << "IoExpander : \"" << name
                          << "\" - IO map error ! Unable to find " << ioName
                          << "\n";
                return false;
            }

            /* Get current value of IO output register */
            int read = i2c_smbus_read_byte_data(
                file, static_cast<uint8_t>(outCtrlBaseAddr + i));
            if (read < 0)
            {
                std::cerr << "IoExpander : \"" << name
                          << "\" - Error: Unable to read data from io expander "
                             "register\n";
                return false;
            }

            std::bitset<8> currVal(read);
            bool writeRequired = false;

            /* Set the bit if the NVMe drive is found in nvmeDrivesInserted, and
             * reset the bit if found in nvmeDrivesRemoved */
            for (uint8_t bit = 0; bit < 8; bit++)
            {
                /* The remove function returns number of elements removed from
                 * list indicating the presence of the drive and also removing
                 * it form the list */
                if (nvmeDrivesInserted.remove(io->second.at(bit)))
                {
                    writeRequired = true;
                    currVal.set(bit);
                    continue;
                }

                if (nvmeDrivesRemoved.remove(io->second.at(bit)))
                {
                    writeRequired = true;
                    currVal.reset(bit);
                }
            }

            if (!writeRequired)
            {
                /* No Write is required as there are no changes */
                continue;
            }

            int ret = i2c_smbus_write_byte_data(
                file, static_cast<uint8_t>(outCtrlBaseAddr + i),
                static_cast<uint8_t>(currVal.to_ulong()));
            if (ret < 0)
            {
                std::cerr << "IoExpander : \"" << name
                          << "\" - Error: Unable to write data to IO expander "
                             "register\n";
                return false;
            }
        }

        return true;
    }

    ~IoExpander()
    {
        if (file >= 0)
        {
            close(file);
        }
    }
};
/***************************** End of Section *******************************/

/****************************************************************************/
/*********************** Global Variables Declarations **********************/
/****************************************************************************/
/* State os Application */
static AppState appState = AppState::idle;

/* Configuration and Components */
static HsbpConfig hsbpConfig;
std::forward_list<ClockBuffer> clockBuffers;
std::forward_list<IoExpander> ioExpanders;

/* Boost IO context and Dbus variables */
boost::asio::io_context io;
auto conn = std::make_shared<sdbusplus::asio::connection>(io);
sdbusplus::asio::object_server objServer(conn);

/* GPIO Lines and GPIO Event Descriptors */
static gpiod::line nvmeLvc3AlertLine;
static boost::asio::posix::stream_descriptor nvmeLvc3AlertEvent(io);
/***************************** End of Section *******************************/

/****************************************************************************/
/********** HSBP Backplane related struct and Global definitions ************/
/****************************************************************************/
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
        ledInterface->register_property(
            ledGroup::asserted, false,
            [weakRef{weak_from_this()}](const bool req, bool& val) {
                auto self = weakRef.lock();
                if (!self)
                {
                    return 0;
                }
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
    Drive(std::string driveName, bool present, bool isOperational, bool nvme,
          bool rebuilding) :
        isNvme(nvme),
        isPresent(present), name(driveName)
    {
        constexpr const char* basePath =
            "/xyz/openbmc_project/inventory/item/drive/";
        itemIface =
            objServer.add_interface(basePath + driveName, inventory::interface);
        itemIface->register_property("Present", isPresent);
        itemIface->register_property("PrettyName", driveName);
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
        logDriveError("Drive " + name);
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
            logDeviceRemoved("Drive", name, serialNumber);
            serialNumber = "N/A";
            serialNumberInitialized = false;
            removeAsset();
        }
        else if (isPresent && !loggedPresent)
        {
            loggedPresent = true;
            logDeviceAdded("Drive", name, serialNumber);
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
    std::string name;
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
            [assetIface{assetInterface}](
                const boost::system::error_code ec,
                const boost::container::flat_map<
                    std::string, std::variant<std::string>>& values) mutable {
                if (ec)
                {
                    std::cerr
                        << "Error getting asset tag from HSBP configuration\n";

                    return;
                }
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

    static std::string zeroPad(const uint8_t val)
    {
        std::ostringstream version;
        version << std::setw(2) << std::setfill('0')
                << static_cast<size_t>(val);
        return version.str();
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

        assetInterface =
            objServer.add_interface(hsbpItemIface->get_object_path(), assetTag);

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

        activationIface =
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
            std::string driveName = boost::replace_all_copy(name, " ", "_") +
                                    "_Drive_" + std::to_string(ii + 1);
            Drive& drive = drives.emplace_back(driveName, isPresent, !isFailed,
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

    bool getInsertedAndRemovedNvmeDrives(
        std::forward_list<std::string>& nvmeDrivesInserted,
        std::forward_list<std::string>& nvmeDrivesRemoved)
    {
        /* Get the current drives status */
        std::bitset<8> currDriveStatus;
        uint8_t nPresence;
        uint8_t nIfdet;

        if (!getPresence(nPresence) || !getIFDET(nIfdet))
        {
            /* Error getting value. Return */
            std::cerr << "Backplane " << name
                      << " failed to get drive status\n";
            return false;
        }

        std::string dbusHsbpName = boost::replace_all_copy(name, " ", "_");
        auto nvmeMap = hsbpConfig.hsbpNvmeMap.find(dbusHsbpName);
        if (nvmeMap == hsbpConfig.hsbpNvmeMap.end())
        {
            std::cerr << "Couldn't get the NVMe Map for the backplane : "
                      << name << "\n";
            return false;
        }

        /* NVMe drives do not assert PRSNTn, and as such do not get reported in
         * "presence" register, but assert ifdet low. This implies for a NVMe
         * drive to be present, corresponding precense bit has to be 0 and idfet
         * has to be 1 (as the values of these regosters are negated: check
         * getPresence() and getIfdet() functions) */
        for (uint8_t bit = 0; bit < 8; bit++)
        {
            if ((nPresence & (1U << bit)) == 0)
            {
                if (nIfdet & (1U << bit))
                {
                    currDriveStatus.set(bit);
                }
            }
        }

        /* Determine Inserted and Removed Drives
         * Prev Bit | Curr Bit | Status
         *    0     |    0     | No Change
         *    0     |    1     | Inserted
         *    1     |    0     | Removed
         *    1     |    1     | No Change
         */
        for (uint8_t index = 0; index < 8; index++)
        {
            /* Inserted */
            if (!prevDriveStatus.test(index) && currDriveStatus.test(index))
            {
                nvmeDrivesInserted.emplace_front(nvmeMap->second.at(index));
                std::cerr << name << " : " << nvmeDrivesInserted.front()
                          << " Inserted !\n";
            }

            /* Removed */
            else if (prevDriveStatus.test(index) &&
                     !currDriveStatus.test(index))
            {
                nvmeDrivesRemoved.emplace_front(nvmeMap->second.at(index));
                std::cerr << name << " : " << nvmeDrivesRemoved.front()
                          << " Removed !\n";
            }
        }

        prevDriveStatus = currDriveStatus;
        return true;
    }

    virtual ~Backplane()
    {
        timer.cancel();
        objServer.remove_interface(hsbpItemIface);
        objServer.remove_interface(versionIface);
        objServer.remove_interface(storageInterface);
        objServer.remove_interface(assetInterface);
        objServer.remove_interface(activationIface);
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
    std::bitset<8> prevDriveStatus;

    int file = -1;

    std::string type;

    std::shared_ptr<sdbusplus::asio::dbus_interface> hsbpItemIface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> versionIface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> storageInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> assetInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> activationIface;
    std::list<Drive> drives;
    std::vector<std::shared_ptr<Led>> leds;
    std::shared_ptr<boost::container::flat_set<Mux>> muxes;
};

/* Global HSBP backplanes and NVMe drives collection */
std::unordered_map<std::string, std::shared_ptr<Backplane>> backplanes;
std::list<Drive> ownerlessDrives; // drives without a backplane
/***************************** End of Section *******************************/

/****************************************************************************/
/***************** Miscellaneous Class/Function Definitions *****************/
/****************************************************************************/
/* The purpose of this class is to sync the code flow. Often times there could
 * be multiple dbus calls which are async, and upon completely finishing all
 * Dbus calls, we need to call next function, or handle the error.
 * When an object of this class goes out of scope, the respective handlers
 * will be called */
class AsyncCallbackHandler
{
    bool errorOccurred = false;
    std::function<void()> onSuccess = nullptr;
    std::function<void()> onError = nullptr;

  public:
    explicit AsyncCallbackHandler(std::function<void()> onSuccessIn,
                                  std::function<void()> onErrorIn) :
        onSuccess(std::move(onSuccessIn)),
        onError(std::move(onErrorIn))
    {
    }

    void setError()
    {
        errorOccurred = true;
    }

    ~AsyncCallbackHandler()
    {
        /* If error occurred flag was set, execute the error handler */
        if (errorOccurred)
        {
            /* Check if Error Handler is defined */
            if (onError)
            {
                onError();
            }

            return;
        }

        /* If Success Handler is present, execute Success Handler */
        if (onSuccess)
        {
            onSuccess();
        }
    }
};

void stopHsbpManager()
{
    std::cerr << __FUNCTION__ << ": Stopping hsbp-manager\n";
    appState = AppState::idle;
    hsbpConfig.clearConfig();
    clockBuffers.clear();
    ioExpanders.clear();
    backplanes.clear();

    io.stop();
}
/***************************** End of Section *******************************/

/****************************************************************************/
/********* HSBP clock enable/disable related Function Definitions ***********/
/****************************************************************************/
void updateHsbpClocks(std::forward_list<std::string>& nvmeDrivesInserted,
                      std::forward_list<std::string>& nvmeDrivesRemoved)
{
    if (appState < AppState::backplanesLoaded)
    {
        std::cerr << "HSBP not initialized ! Cancelling Clock Update ! \n";
        return;
    }

    std::cerr << "Updating HSBP drive clocks ...\n";

    /* Loop through all clock buffers and try to update the clocks (this will be
     * done if the mode of operation of the clock buffer is SMBus) */
    for (auto& clockBuffer : clockBuffers)
    {
        if (!clockBuffer.enableDisableClock(nvmeDrivesInserted,
                                            nvmeDrivesRemoved))
        {
            std::cerr << "Error Occurred while setting the clock in \""
                      << clockBuffer.getName() << "\"\n";
        }
    }

    /* If there are drives yet to be updated, check all the IO Expanders in case
     * they are mapped to the drives and enable the respective IO */
    if (!nvmeDrivesInserted.empty() || !nvmeDrivesRemoved.empty())
    {
        for (auto& ioExpander : ioExpanders)
        {
            if (!ioExpander.enableDisableOuput(nvmeDrivesInserted,
                                               nvmeDrivesRemoved))
            {
                std::cerr << "Error Occurred while setting the IO in \""
                          << ioExpander.getName() << "\"\n";
            }
        }
    }

    /* If there are drives still left, then one or more drives clock
     * enable/diable failed. There is a possibility of improper mapping or
     * current communication with the device failed */
    if (!nvmeDrivesInserted.empty() || !nvmeDrivesRemoved.empty())
    {
        std::cerr << "Critical Error !!!\nMapping issue detected !\n";

        if (!nvmeDrivesInserted.empty())
        {
            std::cerr << "The clock enable failed for : ";
            for (auto& nvme1 : nvmeDrivesInserted)
            {
                std::cerr << nvme1 << ", ";
            }
            std::cerr << "\n";
        }

        if (!nvmeDrivesRemoved.empty())
        {
            std::cerr << "The clock disable failed for : ";
            for (auto& nvme1 : nvmeDrivesRemoved)
            {
                std::cerr << nvme1 << ", ";
            }
            std::cerr << "\n";
        }
    }
}

void scanHsbpDrives(bool& hsbpDriveScanInProgress)
{
    std::cerr << __FUNCTION__ << ": Scanning HSBP drives status ...\n";

    /* List variables to store the drives Inserted/Removed */
    std::forward_list<std::string> nvmeDrivesInserted;
    std::forward_list<std::string> nvmeDrivesRemoved;

    /* Loop through each backplane present and get the list of inserted/removed
     * drives */
    for (auto& [name, backplane] : backplanes)
    {
        backplane->getInsertedAndRemovedNvmeDrives(nvmeDrivesInserted,
                                                   nvmeDrivesRemoved);
    }

    if (!nvmeDrivesInserted.empty() || !nvmeDrivesRemoved.empty())
    {
        updateHsbpClocks(nvmeDrivesInserted, nvmeDrivesRemoved);
    }

    std::cerr << __FUNCTION__ << ": Scanning HSBP drives Completed\n";

    hsbpDriveScanInProgress = false;
}

void checkHsbpDrivesStatus()
{
    static bool hsbpDriveScanInProgress = false;
    static bool hsbpDriveRescanInQueue = false;

    if (appState < AppState::backplanesLoaded)
    {
        std::cerr << __FUNCTION__
                  << ": HSBP not initialized ! Cancelling scan of HSBP drives "
                     "status ! \n";
        return;
    }

    if (hsbpDriveScanInProgress)
    {
        /* Scan and Clock Update already in progress. Try again after sometime.
         * This event can occur due to the GPIO interrupt */
        std::cerr << __FUNCTION__
                  << ": HSBP Drives Scan is already in progress\n";
        if (hsbpDriveRescanInQueue)
        {
            /* There is already a Re-Scan in queue. No need to create multiple
             * rescans */
            return;
        }

        hsbpDriveRescanInQueue = true;

        std::cerr << __FUNCTION__ << ": Queuing the Scan \n";

        auto driveScanTimer = std::make_shared<boost::asio::steady_timer>(io);
        driveScanTimer->expires_after(std::chrono::seconds(1));
        driveScanTimer->async_wait(
            [driveScanTimer](const boost::system::error_code ec) {
                if (ec == boost::asio::error::operation_aborted)
                {
                    // Timer was Aborted
                    return;
                }
                else if (ec)
                {
                    std::cerr << "driveScanTimer: Timer error" << ec.message()
                              << "\n";
                    return;
                }
                hsbpDriveRescanInQueue = false;
                checkHsbpDrivesStatus();
            });

        return;
    }

    hsbpDriveScanInProgress = true;

    /* Post the scan to IO queue and return from here. This enables capturing
     * next GPIO event if any */
    boost::asio::post(io, []() { scanHsbpDrives(hsbpDriveScanInProgress); });
}

void disableAllClocks()
{
    /* Loop through all clock buffers and try to disable all clocks (this will
     * be done if the mode of operation of the clock buffer is SMBus) */
    for (auto& clockBuffer : clockBuffers)
    {
        if (!clockBuffer.disableClocks())
        {
            std::cerr << "Error Occurred while diabling the clock in \""
                      << clockBuffer.getName() << "\"\n";
        }
    }

    /* Loop through all IO Controllers and try to update the output to disable
     * all clocks */
    for (auto& ioExpander : ioExpanders)
    {
        if (!ioExpander.disableIO())
        {
            std::cerr << "Error Occurred while disabling the IO in \""
                      << ioExpander.getName() << "\"\n";
        }
    }
}
/***************************** End of Section *******************************/

/****************************************************************************/
/********** Backplanes and NVMe drives related Function Definitions *********/
/****************************************************************************/
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
    appState = AppState::loadingDrives;

    /* Setup a callback to be called once the assets are populated completely or
     * fallback to error handler */
    auto drivesLoadedCallback = std::make_shared<AsyncCallbackHandler>(
        []() {
            appState = AppState::drivesLoaded;
            std::cerr << "Drives Updated !\n";
        },
        []() {
            // TODO: Handle this error if needed
            appState = AppState::backplanesLoaded;
            std::cerr << "An error occured ! Drives load failed \n";
        });

    conn->async_method_call(
        [drivesLoadedCallback](const boost::system::error_code ec,
                               const GetSubTreeType& subtree) {
            if (ec)
            {
                std::cerr << __FUNCTION__ << ": Error contacting mapper "
                          << ec.message() << "\n";
                drivesLoadedCallback->setError();
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
                    [path, drivesLoadedCallback](
                        const boost::system::error_code ec2,
                        const boost::container::flat_map<
                            std::string, std::variant<uint64_t, std::string>>&
                            values) {
                        if (ec2)
                        {
                            std::cerr << __FUNCTION__
                                      << ": Error Getting Config "
                                      << ec2.message() << " "
                                      << "\n";
                            drivesLoadedCallback->setError();
                            return;
                        }
                        auto findBus = values.find("Bus");

                        if (findBus == values.end())
                        {
                            std::cerr << __FUNCTION__
                                      << ": Illegal interface at " << path
                                      << "\n";
                            drivesLoadedCallback->setError();
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
                            drivesLoadedCallback->setError();
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
                            drivesLoadedCallback->setError();
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
                            std::cerr << __FUNCTION__
                                      << ": Unable to open name file of bus "
                                      << muxBus << "\n";
                            drivesLoadedCallback->setError();
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
                            std::cerr << __FUNCTION__
                                      << ": Illegal name file on bus " << muxBus
                                      << "\n";
                        }

                        std::string indexStr =
                            nameStr.substr(findId + prefix.size(), 1);

                        size_t driveIndex = std::stoi(indexStr);

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
                            if (parent)
                            {
                                /* Found the backplane. No need to proceed
                                 * further */
                                break;
                            }
                        }

                        // assume its a M.2 or something without a hsbp
                        if (parent == nullptr)
                        {
                            std::string driveName =
                                "Drive_" + std::to_string(getDriveCount() + 1);
                            auto& drive = ownerlessDrives.emplace_back(
                                driveName, true, true, true, false);
                            drive.createAsset(assetInventory);
                            return;
                        }

                        driveIndex += muxIndex;

                        if (parent->drives.size() <= driveIndex)
                        {
                            std::cerr << __FUNCTION__
                                      << ": Illegal drive index at " << path
                                      << " " << driveIndex << "\n";
                            drivesLoadedCallback->setError();
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
        0, std::array<const char*, 1>{nvmeIntf});
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
                std::cerr << __FUNCTION__ << ": Error contacting mapper "
                          << ec.message() << "\n";
                return;
            }
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
                    std::cerr << __FUNCTION__ << ": Cannot get mux type\n";
                    continue;
                }

                conn->async_method_call(
                    [path, muxes, index](
                        const boost::system::error_code ec2,
                        const boost::container::flat_map<
                            std::string,
                            std::variant<uint64_t, std::vector<std::string>>>&
                            values) {
                        if (ec2)
                        {
                            std::cerr << __FUNCTION__
                                      << ": Error Getting Config "
                                      << ec2.message() << "\n";
                            return;
                        }
                        auto findBus = values.find("Bus");
                        auto findAddress = values.find("Address");
                        auto findChannelNames = values.find("ChannelNames");
                        if (findBus == values.end() ||
                            findAddress == values.end())
                        {
                            std::cerr << __FUNCTION__
                                      << ": Illegal configuration at " << path
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
                    },
                    owner, path, "org.freedesktop.DBus.Properties", "GetAll",
                    *interface);
                index++;
            }
        },
        mapper::busName, mapper::path, mapper::interface, mapper::subtree,
        rootPath, 1, muxTypes);
}

void populateHsbpBackplanes(
    const std::shared_ptr<AsyncCallbackHandler>& backplanesLoadedCallback)
{
    std::cerr << __FUNCTION__ << ": Scanning Backplanes ...\n";
    appState = AppState::loadingBackplanes;
    backplanes.clear();

    conn->async_method_call(
        [backplanesLoadedCallback](const boost::system::error_code ec,
                                   const GetSubTreeType& subtree) {
            if (ec)
            {
                std::cerr << __FUNCTION__ << ": Error contacting mapper "
                          << ec.message() << "\n";
                backplanesLoadedCallback->setError();
                return;
            }

            if (subtree.empty())
            {
                /* There wer no HSBP's detected. set teh state back to
                 * componentsLoaded so that on backplane match event, the
                 * process can start again */
                appState = AppState::componentsLoaded;
                std::cerr << __FUNCTION__ << ": No HSBPs Detected....\n";

                /* Set the components to their default value. This will result
                 * in diabling the clock outputs. As there are no drives
                 * detected, all the Clocks can be disabled */
                disableAllClocks();
                return;
            }

            for (const auto& [path, objDict] : subtree)
            {
                if (objDict.empty())
                {
                    std::cerr << __FUNCTION__
                              << ": Subtree data "
                                 "corrupted !\n";
                    backplanesLoadedCallback->setError();
                    return;
                }

                const std::string& owner = objDict.begin()->first;
                conn->async_method_call(
                    [backplanesLoadedCallback, path,
                     owner](const boost::system::error_code ec2,
                            const boost::container::flat_map<
                                std::string, BasicVariantType>& resp) {
                        if (ec2)
                        {
                            std::cerr << __FUNCTION__
                                      << ": Error Getting Config "
                                      << ec2.message() << "\n";
                            backplanesLoadedCallback->setError();
                            return;
                        }
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
                            std::cerr << __FUNCTION__
                                      << ": Illegal configuration at " << path
                                      << "\n";
                            backplanesLoadedCallback->setError();
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
                    hsbpCpldInft);
            }
        },
        mapper::busName, mapper::path, mapper::interface, mapper::subtree, "/",
        0, std::array<const char*, 1>{hsbpCpldInft});
}

void setUpBackplanesAndDrives()
{
    static bool backplanesScanInProgress = false;
    static bool backplanesRescanInQueue = false;

    if (appState < AppState::componentsLoaded)
    {
        std::cerr << __FUNCTION__
                  << ": Components are not initialized ! Cancelling scan of "
                     "Backplanes ! \n";
        return;
    }

    if (backplanesScanInProgress)
    {
        std::cerr << __FUNCTION__
                  << ": Backplanes Scan is already in progress\n";
        if (backplanesRescanInQueue)
        {
            /* There is already a Re-Scan in queue. No need to create multiple
             * rescans */
            return;
        }

        backplanesRescanInQueue = true;

        std::cerr << __FUNCTION__ << ": Queuing the Backplane Scan \n";

        auto backplaneScanTimer =
            std::make_shared<boost::asio::steady_timer>(io);
        backplaneScanTimer->expires_after(std::chrono::seconds(1));
        backplaneScanTimer->async_wait(
            [backplaneScanTimer](const boost::system::error_code ec) {
                if (ec == boost::asio::error::operation_aborted)
                {
                    // Timer was Aborted
                    return;
                }
                else if (ec)
                {
                    std::cerr << "backplaneScanTimer: Timer error"
                              << ec.message() << "\n";
                    return;
                }
                backplanesRescanInQueue = false;
                setUpBackplanesAndDrives();
            });

        return;
    }

    backplanesScanInProgress = true;

    /* Set Callback to be called once backplanes are populated to call
     * updateAssets() and checkHsbpDrivesStatus() or handle error scnenario */
    auto backplanesLoadedCallback = std::make_shared<AsyncCallbackHandler>(
        []() {
            /* If no HSBP's were detected, the state changes to
             * componentsLoaded. Proceed further only if state was
             * loadingBackplanes */
            if (appState != AppState::loadingBackplanes)
            {
                backplanesScanInProgress = false;
                return;
            }

            /* If there is a ReScan in the Queue, dont proceed further. Load the
             * Backplanes again and then proceed further */
            if (backplanesRescanInQueue)
            {
                backplanesScanInProgress = false;
                return;
            }

            appState = AppState::backplanesLoaded;
            std::cerr << __FUNCTION__ << ": Backplanes Loaded...\n";

            checkHsbpDrivesStatus();
            updateAssets();
            backplanesScanInProgress = false;
        },
        []() {
            /* Loading Backplanes is an important step. If the load failed due
             * to an error, stop the app so that restart cant be triggerred */
            std::cerr << "Backplanes couldn't be loaded due to an error !...\n";
            appState = AppState::idle;
            backplanesScanInProgress = false;
            stopHsbpManager();
        });

    populateHsbpBackplanes(backplanesLoadedCallback);
}

void setupBackplanesAndDrivesMatch()
{
    static auto backplaneMatch = std::make_unique<sdbusplus::bus::match_t>(
        *conn,
        "sender='xyz.openbmc_project.EntityManager', type='signal', "
        "member='PropertiesChanged', "
        "interface='org.freedesktop.DBus.Properties', "
        "path_namespace='/xyz/openbmc_project/inventory/system/board', arg0='" +
            std::string(hsbpCpldInft) + "'",
        [](sdbusplus::message_t& msg) {
            std::string intfName;
            boost::container::flat_map<std::string, BasicVariantType> values;
            msg.read(intfName, values);

            /* This match will be triggered for each of the property being set
             * under the hsbpCpldInft interface. Call the loader only on one
             * property say "name". This will avoid multiple calls to populate
             * function
             */
            for (const auto& [key, value] : values)
            {
                if (key == "Name")
                {
                    /* This match will be triggered when ever there is a
                     * addition/removal of HSBP backplane. At this stage, all
                     * the HSBP's need to be populated again and also assets
                     * have to be re-discovered. So, setting state to
                     * componentsLoaded and calling setUpBackplanesAndDrives()
                     * only if configuration and components loading was
                     * completed */
                    if (appState < AppState::componentsLoaded)
                    {
                        /* Configuration is not loaded yet. Backplanes will be
                         * loaded
                         * once configuration and components are loaded. */
                        std::cerr << __FUNCTION__
                                  << ": Discarding Backplane match\n";
                        return;
                    }

                    appState = AppState::componentsLoaded;

                    /* We will call the function after a small delay to let all
                     * the properties to be intialized */
                    auto backplaneTimer =
                        std::make_shared<boost::asio::steady_timer>(io);
                    backplaneTimer->expires_after(std::chrono::seconds(2));
                    backplaneTimer->async_wait(
                        [backplaneTimer](const boost::system::error_code ec) {
                            if (ec == boost::asio::error::operation_aborted)
                            {
                                return;
                            }
                            else if (ec)
                            {
                                std::cerr << "backplaneTimer: Timer error"
                                          << ec.message() << "\n";
                                return;
                            }
                            setUpBackplanesAndDrives();
                        });
                }
            }
        });

    static auto drivesMatch = std::make_unique<sdbusplus::bus::match_t>(
        *conn,
        "sender='xyz.openbmc_project.EntityManager', type='signal', "
        "member='PropertiesChanged', "
        "interface='org.freedesktop.DBus.Properties', arg0='" +
            std::string(nvmeIntf) + "'",
        [](sdbusplus::message_t& msg) {
            std::string intfName;
            boost::container::flat_map<std::string, BasicVariantType> values;
            msg.read(intfName, values);

            /* This match will be triggered for each of the property being set
             * under the nvmeIntf interface. Call the loader only on one
             * property say "name". This will avoid multiple calls to populate
             * function
             */
            for (const auto& [key, value] : values)
            {
                if (key == "Name")
                {
                    /* This match will be triggered when ever there is a
                     * addition/removal of drives. At this stage only assets
                     * have to be re-discovered. So, setting state to
                     * backplanesLoaded and calling updateAssets() only if all
                     * previous states are completed */
                    if (appState < AppState::backplanesLoaded)
                    {
                        /* Configuration is not loaded yet. Drives will be
                         * loaded once
                         * configuration, components and backplanes are loaded.
                         */
                        std::cerr << __FUNCTION__
                                  << ": Discarding Drive match\n";
                        return;
                    }

                    appState = AppState::backplanesLoaded;

                    /* We will call the function after a small delay to let all
                     * the properties to be intialized */
                    auto driveTimer =
                        std::make_shared<boost::asio::steady_timer>(io);
                    driveTimer->expires_after(std::chrono::seconds(2));
                    driveTimer->async_wait(
                        [driveTimer](const boost::system::error_code ec) {
                            if (ec == boost::asio::error::operation_aborted)
                            {
                                return;
                            }
                            else if (ec)
                            {
                                std::cerr << "driveTimer: Timer error"
                                          << ec.message() << "\n";
                                return;
                            }
                            updateAssets();
                        });
                }
            }
        });
}
/***************************** End of Section *******************************/

/****************************************************************************/
/******************* Components related Function Definitions ****************/
/****************************************************************************/
bool verifyComponentsLoaded()
{
    std::cerr << __FUNCTION__ << ": Verifying all Components...\n";

    /* Loop through all clock buffers */
    for (auto& clockBuffer : clockBuffers)
    {
        if (!clockBuffer.isInitialized())
        {
            std::cerr << "Critical Error: Initializing \""
                      << clockBuffer.getName() << "\" failed\n";
            return false;
        }
    }

    /* Loop through all IO Expanders */
    for (auto& ioExpander : ioExpanders)
    {
        if (!ioExpander.isInitialized())
        {
            std::cerr << "Critical Error: Initializing \""
                      << ioExpander.getName() << "\" failed\n";
            return false;
        }
    }

    std::cerr << __FUNCTION__ << ": Verifying Components Complete\n";

    return true;
}
/***************************** End of Section *******************************/

/****************************************************************************/
/****************** IO expander related Function Definitions ****************/
/****************************************************************************/
void loadIoExpanderInfo(
    const std::shared_ptr<AsyncCallbackHandler>& componentsLoadedCallback)
{
    appState = AppState::loadingComponents;

    /* Clear global ioExpanders to start off */
    ioExpanders.clear();

    conn->async_method_call(
        [componentsLoadedCallback](const boost::system::error_code ec,
                                   const GetSubTreeType& subtree) {
            if (ec)
            {
                std::cerr << __FUNCTION__ << ": Error contacting mapper "
                          << ec.message() << "\n";
                componentsLoadedCallback->setError();
                return;
            }

            for (auto& [path, objDict] : subtree)
            {

                if (objDict.empty())
                {
                    std::cerr << __FUNCTION__ << ": Subtree data corrupted !\n";
                    componentsLoadedCallback->setError();
                    return;
                }

                /* Ideally there would be only one element in objDict as only
                 * one service exposes it and there would be only one interface
                 * so it is safe to directly read them without loop */
                const std::string& service = objDict.begin()->first;
                const std::string& intf = objDict.begin()->second.front();

                conn->async_method_call(
                    [componentsLoadedCallback](
                        const boost::system::error_code er,
                        const boost::container::flat_map<
                            std::string, BasicVariantType>& resp) {
                        if (er)
                        {
                            std::cerr << __FUNCTION__
                                      << ": Error Getting "
                                         "Config "
                                      << er.message() << "\n";
                            componentsLoadedCallback->setError();
                            return;
                        }

                        std::optional<uint64_t> bus;
                        std::optional<uint64_t> address;
                        std::optional<uint64_t> confIORegAddr;
                        std::optional<uint64_t> outCtrlBaseAddr;
                        std::optional<uint64_t> outCtrlByteCount;
                        std::unordered_map<std::string,
                                           std::vector<std::string>>
                            ioMap;
                        std::optional<std::string> name;
                        std::optional<std::string> type;

                        /* Loop through to get all IO Expander properties */
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
                            else if (key == "ConfIORegAddr")
                            {
                                confIORegAddr = std::get<uint64_t>(value);
                            }
                            else if (key == "OutCtrlBaseAddr")
                            {
                                outCtrlBaseAddr = std::get<uint64_t>(value);
                            }
                            else if (key == "OutCtrlByteCount")
                            {
                                outCtrlByteCount = std::get<uint64_t>(value);
                            }
                            else if (key == "Name")
                            {
                                name = std::get<std::string>(value);
                            }
                            else if (key == "Type")
                            {
                                type = std::get<std::string>(value);
                            }
                            else if (key.starts_with("IO"))
                            {
                                std::optional<std::vector<std::string>> outList;
                                outList = std::get<NvmeMapping>(value);
                                if (!outList)
                                {
                                    break;
                                }
                                ioMap.try_emplace(key, *outList);
                            }
                        }

                        /* Verify if all properties were defined */
                        if (!bus || !address || !confIORegAddr ||
                            !outCtrlBaseAddr || !outCtrlByteCount || !name)
                        {
                            std::cerr << __FUNCTION__
                                      << ": Incomplete "
                                         "Clock Buffer definition !! \n";
                            componentsLoadedCallback->setError();
                            return;
                        }

                        /* Check if we were able to get byteMap correctly */
                        if ((*outCtrlByteCount) != ioMap.size())
                        {
                            std::cerr << "loadIoExpanderInfo(): Incomplete "
                                         "IO Map !! \n";
                            componentsLoadedCallback->setError();
                            return;
                        }

                        /* Create IO expander object and add it to global
                         * ioExpanders vector */
                        ioExpanders.emplace_front(
                            *bus, *address, *confIORegAddr, *outCtrlBaseAddr,
                            *outCtrlByteCount, ioMap, *name, *type);
                    },
                    service, path, "org.freedesktop.DBus.Properties", "GetAll",
                    intf);
            }
        },
        mapper::busName, mapper::path, mapper::interface, mapper::subtree, "/",
        0, hsbpConfig.ioExpanderTypes);
}
/***************************** End of Section *******************************/

/****************************************************************************/
/***************** Clock buffer related Function Definitions ****************/
/****************************************************************************/
void loadClockBufferInfo(
    const std::shared_ptr<AsyncCallbackHandler>& componentsLoadedCallback)
{
    appState = AppState::loadingComponents;

    /* Clear global clockBuffers to start off */
    clockBuffers.clear();

    conn->async_method_call(
        [componentsLoadedCallback](const boost::system::error_code ec,
                                   const GetSubTreeType& subtree) {
            if (ec)
            {
                std::cerr << __FUNCTION__ << ": Error contacting mapper "
                          << ec.message() << "\n";
                componentsLoadedCallback->setError();
                return;
            }

            for (auto& [path, objDict] : subtree)
            {

                if (objDict.empty())
                {
                    std::cerr << __FUNCTION__ << ": Subtree data corrupted !\n";
                    componentsLoadedCallback->setError();
                    return;
                }

                /* Ideally there would be only one element in objDict as only
                 * one service exposes it and there would be only one interface
                 * so it is safe to directly read them without loop */
                const std::string& service = objDict.begin()->first;
                const std::string& intf = objDict.begin()->second.front();

                conn->async_method_call(
                    [componentsLoadedCallback](
                        const boost::system::error_code er,
                        const boost::container::flat_map<
                            std::string, BasicVariantType>& resp) {
                        if (er)
                        {
                            std::cerr << __FUNCTION__
                                      << ": Error Getting "
                                         "Config "
                                      << er.message() << "\n";
                            componentsLoadedCallback->setError();
                            return;
                        }

                        std::optional<uint64_t> bus;
                        std::optional<uint64_t> address;
                        std::optional<std::string> mode;
                        std::optional<uint64_t> outCtrlBaseAddr;
                        std::optional<uint64_t> outCtrlByteCount;
                        std::unordered_map<std::string,
                                           std::vector<std::string>>
                            byteMap;
                        std::optional<std::string> name;
                        std::optional<std::string> type;

                        /* Loop through to get all Clock Buffer properties */
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
                            else if (key == "Mode")
                            {
                                mode = std::get<std::string>(value);
                            }
                            else if (key == "OutCtrlBaseAddr")
                            {
                                outCtrlBaseAddr = std::get<uint64_t>(value);
                            }
                            else if (key == "OutCtrlByteCount")
                            {
                                outCtrlByteCount = std::get<uint64_t>(value);
                            }
                            else if (key == "Name")
                            {
                                name = std::get<std::string>(value);
                            }
                            else if (key == "Type")
                            {
                                type = std::get<std::string>(value);
                            }
                            else if (key.starts_with("Byte"))
                            {
                                std::optional<std::vector<std::string>>
                                    byteList;
                                byteList = std::get<NvmeMapping>(value);
                                if (!byteList)
                                {
                                    break;
                                }
                                byteMap.try_emplace(key, *byteList);
                            }
                        }

                        /* Verify if all properties were defined */
                        if (!bus || !address || !mode || !outCtrlBaseAddr ||
                            !outCtrlByteCount || !name)
                        {
                            std::cerr << __FUNCTION__
                                      << ": Incomplete "
                                         "Clock Buffer definition !! \n";
                            componentsLoadedCallback->setError();
                            return;
                        }

                        /* Check if we were able to get byteMap correctly */
                        if ((*outCtrlByteCount) != byteMap.size())
                        {
                            std::cerr << __FUNCTION__
                                      << ": Incomplete "
                                         "Byte Map !! \n";
                            componentsLoadedCallback->setError();
                            return;
                        }

                        /* Create clock buffer object and add it to global
                         * clockBuffers vector */
                        clockBuffers.emplace_front(
                            *bus, *address, *mode, *outCtrlBaseAddr,
                            *outCtrlByteCount, byteMap, *name, *type);
                    },
                    service, path, "org.freedesktop.DBus.Properties", "GetAll",
                    intf);
            }
        },
        mapper::busName, mapper::path, mapper::interface, mapper::subtree, "/",
        0, hsbpConfig.clockBufferTypes);
}
/***************************** End of Section *******************************/

/****************************************************************************/
/***************** HSBP Config related Function Definitions *****************/
/****************************************************************************/
void loadHsbpConfig()
{
    appState = AppState::loadingHsbpConfig;

    conn->async_method_call(
        [](const boost::system::error_code ec, const GetSubTreeType& subtree) {
            if (ec)
            {
                std::cerr << __FUNCTION__ << ": Error contacting mapper "
                          << ec.message() << "\n";
                return;
            }

            if (subtree.empty())
            {
                /* Entity manager is either still loading the configuration or
                 * failed to load. In either way, return from here as the dbus
                 * match will take care */
                std::cerr << __FUNCTION__ << ": No configuration detected !!\n";
                return;
            }

            /* There should be only one HSBP Configureation exposed */
            if (subtree.size() != 1)
            {
                std::cerr << __FUNCTION__
                          << ": Multiple configurations "
                             "detected !!\n";
                /* Critical Error. Stop Application */
                stopHsbpManager();
                return;
            }

            auto& path = subtree.begin()->first;
            auto& objDict = subtree.begin()->second;

            if (objDict.empty())
            {
                /* Critical Error. Stop Application */
                std::cerr << __FUNCTION__ << ": Subtree data corrupted !\n";
                stopHsbpManager();
                return;
            }

            const std::string& service = objDict.begin()->first;

            conn->async_method_call(
                [](const boost::system::error_code er,
                   const boost::container::flat_map<std::string,
                                                    BasicVariantType>& resp) {
                    if (er)
                    {
                        std::cerr << __FUNCTION__ << ": Error Getting Config "
                                  << er.message() << "\n";
                        /* Critical Error. Stop Application */
                        stopHsbpManager();
                        return;
                    }

                    std::optional<uint64_t> rootI2cBus;
                    std::optional<std::vector<std::string>> supportedHsbps;
                    std::optional<std::vector<std::string>> clockBufferTypes;
                    std::optional<std::vector<std::string>> ioExpanderTypes;

                    /* Loop through to get root i2c bus and list of supported
                     * HSBPs */
                    for (const auto& [key, value] : resp)
                    {
                        if (key == "HsbpSupported")
                        {
                            supportedHsbps =
                                std::get<std::vector<std::string>>(value);
                        }
                        else if (key == "RootI2cBus")
                        {
                            rootI2cBus = std::get<uint64_t>(value);
                        }
                        else if (key == "ClockBuffer")
                        {
                            clockBufferTypes =
                                std::get<std::vector<std::string>>(value);
                        }
                        else if (key == "IoExpander")
                        {
                            ioExpanderTypes =
                                std::get<std::vector<std::string>>(value);
                        }
                    }

                    /* Verify if i2c bus, supported HSBP's and clock buffers
                     * were defined (IO Expanders are optional) */
                    if (!rootI2cBus || !supportedHsbps || !clockBufferTypes)
                    {
                        std::cerr << __FUNCTION__
                                  << ": Incomplete HSBP "
                                     "configuration !! \n";
                        /* Critical Error. Stop Application */
                        stopHsbpManager();
                        return;
                    }

                    /* Clear and Load all details to global hsbp configuration
                     * variable */
                    hsbpConfig.clearConfig();
                    hsbpConfig.rootBus = *rootI2cBus;
                    hsbpConfig.supportedHsbps = std::move(*supportedHsbps);

                    for (auto& clkBuffType : *clockBufferTypes)
                    {
                        hsbpConfig.clockBufferTypes.emplace_back(
                            "xyz.openbmc_project.Configuration." + clkBuffType);
                    }

                    if (ioExpanderTypes)
                    {
                        for (auto& ioCntrType : *ioExpanderTypes)
                        {
                            hsbpConfig.ioExpanderTypes.emplace_back(
                                "xyz.openbmc_project.Configuration." +
                                ioCntrType);
                        }
                    }

                    /* Loop through to get HSBP-NVME map and Components map
                     * details */
                    uint8_t hsbpMapCount = 0;
                    for (const auto& [key, value] : resp)
                    {
                        if (std::find(hsbpConfig.supportedHsbps.begin(),
                                      hsbpConfig.supportedHsbps.end(),
                                      key) != hsbpConfig.supportedHsbps.end())
                        {
                            std::optional<std::vector<std::string>> hsbpMap;
                            hsbpMap = std::get<NvmeMapping>(value);
                            if (!hsbpMap)
                            {
                                break;
                            }
                            hsbpConfig.hsbpNvmeMap.try_emplace(key, *hsbpMap);
                            hsbpMapCount++;
                        }
                    }

                    /* Check if we were able to get all the HSBP-NVMe maps */
                    if (hsbpConfig.supportedHsbps.size() != hsbpMapCount)
                    {
                        std::cerr << __FUNCTION__
                                  << ": Incomplete HSBP Map "
                                     "details !! \n";
                        /* Critical Error. Stop Application */
                        stopHsbpManager();
                        return;
                    }

                    /* HSBP configuration is loaded */
                    appState = AppState::hsbpConfigLoaded;
                    std::cerr << "HSBP Config loaded !\n";

                    /* Get Clock buffers and IO expander details. Create shared
                     * object of AsyncCallbackHandler with success and error
                     * callback */
                    auto componentsLoadedCallback = std::make_shared<
                        AsyncCallbackHandler>(
                        []() {
                            /* Verify if all components were initialized without
                             * errors */
                            if (!verifyComponentsLoaded())
                            {
                                /* The application cannot proceed further as
                                 * components initialization failed. App needs
                                 * Restart */
                                appState = AppState::idle;
                                std::cerr
                                    << "One or more Componenets initialization "
                                       "failed !! Restart Required !\n";
                                stopHsbpManager();
                            }

                            appState = AppState::componentsLoaded;
                            setUpBackplanesAndDrives();
                        },
                        []() {
                            /* The application cannot proceed further as
                             * components load failed. App needs Restart */
                            appState = AppState::idle;
                            std::cerr << "Loading Componenets failed !! "
                                         "Restart Required !\n";
                            stopHsbpManager();
                        });

                    loadClockBufferInfo(componentsLoadedCallback);

                    if (ioExpanderTypes)
                    {
                        loadIoExpanderInfo(componentsLoadedCallback);
                    }
                },
                service, path, "org.freedesktop.DBus.Properties", "GetAll",
                hsbpConfigIntf);
        },
        mapper::busName, mapper::path, mapper::interface, mapper::subtree, "/",
        0, std::array<const char*, 1>{hsbpConfigIntf});
}

void setupHsbpConfigMatch()
{
    static auto hsbpConfigMatch = std::make_unique<sdbusplus::bus::match_t>(
        *conn,
        "sender='xyz.openbmc_project.EntityManager', type='signal', "
        "member='PropertiesChanged', "
        "interface='org.freedesktop.DBus.Properties', "
        "path_namespace='/xyz/openbmc_project/inventory/system/board', arg0='" +
            std::string(hsbpConfigIntf) + "'",
        [](sdbusplus::message_t& msg) {
            std::string intfName;
            boost::container::flat_map<std::string, BasicVariantType> values;
            msg.read(intfName, values);

            /* This match will be triggered for each of the property being set
             * under the hsbpConfig interface. "HsbpSupported" is one of the
             * important property which will enable us to read other properties.
             * So, when the match event occurs for "HsbpSupported" property
             * being set, we will call "loadHsbpConfig()" If the control has
             * come here, its either the first initialization or entity-manager
             * reload. So, we will reset the state to uninitialized
             */
            for (const auto& [key, value] : values)
            {
                if (key == "HsbpSupported")
                {
                    /* Configuration change detected, change the state to stop
                     * other processing */
                    appState = AppState::idle;

                    /* We will call the function after a small delay to let all
                     * the properties to be intialized */
                    auto loadTimer =
                        std::make_shared<boost::asio::steady_timer>(io);
                    loadTimer->expires_after(std::chrono::seconds(1));
                    loadTimer->async_wait(
                        [loadTimer](const boost::system::error_code ec) {
                            if (ec == boost::asio::error::operation_aborted)
                            {
                                return;
                            }
                            else if (ec)
                            {
                                std::cerr << __FUNCTION__ << ": Timer error"
                                          << ec.message() << "\n";
                                if (hsbpConfig.supportedHsbps.empty())
                                {
                                    /* Critical Error as none of the
                                     * configuration was loaded and timer
                                     * failed. Stop the application */
                                    stopHsbpManager();
                                }
                                return;
                            }
                            loadHsbpConfig();
                        });
                }
            }
        });
}
/***************************** End of Section *******************************/

/****************************************************************************/
/***************** GPIO Events related Function Definitions *****************/
/****************************************************************************/
static void nvmeLvc3AlertHandler()
{
    /* If the state is not backplanesLoaded, we ignore the GPIO event as we
     * cannot communicate to the backplanes yet */
    if (appState < AppState::backplanesLoaded)
    {
        std::cerr << __FUNCTION__
                  << ": HSBP not initialized ! Dropping the interrupt ! \n";
        return;
    }

    /* This GPIO event only indicates the addition or removal of drive to either
     * of CPU. The backplanes detected need to be scanned and detect which drive
     * has been added/removed and enable/diable clock accordingly */
    gpiod::line_event gpioLineEvent = nvmeLvc3AlertLine.event_read();

    if (gpioLineEvent.event_type == gpiod::line_event::FALLING_EDGE)
    {
        /* Check for HSBP Drives status to determine if any new drive has been
         * added/removed and update clocks accordingly */
        checkHsbpDrivesStatus();
    }

    nvmeLvc3AlertEvent.async_wait(
        boost::asio::posix::stream_descriptor::wait_read,
        [](const boost::system::error_code ec) {
            if (ec)
            {
                std::cerr << __FUNCTION__
                          << ": nvmealert event error: " << ec.message()
                          << "\n";
            }
            nvmeLvc3AlertHandler();
        });
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
        std::cerr << __FUNCTION__ << ": Failed to find the " << name
                  << " line\n";
        return false;
    }

    try
    {
        gpioLine.request(
            {"hsbp-manager", gpiod::line_request::EVENT_BOTH_EDGES, 0});
    }
    catch (std::exception&)
    {
        std::cerr << __FUNCTION__ << ": Failed to request events for " << name
                  << "\n";
        return false;
    }

    int gpioLineFd = gpioLine.event_get_fd();
    if (gpioLineFd < 0)
    {
        std::cerr << __FUNCTION__ << ": Failed to get " << name << " fd\n";
        return false;
    }

    gpioEventDescriptor.assign(gpioLineFd);

    gpioEventDescriptor.async_wait(
        boost::asio::posix::stream_descriptor::wait_read,
        [&name, handler](const boost::system::error_code ec) {
            if (ec)
            {
                std::cerr << __FUNCTION__ << ": " << name
                          << " fd handler error: " << ec.message() << "\n";
                return;
            }
            handler();
        });
    return true;
}
/***************************** End of Section *******************************/

int main()
{
    std::cerr << "******* Starting hsbp-manager *******\n";

    /* Set the Dbus name */
    conn->request_name(busName);

    /* Add interface for storage inventory */
    objServer.add_interface("/xyz/openbmc_project/inventory/item/storage",
                            "xyz.openbmc_project.inventory.item.storage");

    /* HSBP initializtion flow:
     * 1. Register GPIO event callback on FM_SMB_BMC_NVME_LVC3_ALERT_N line
     * 2. Set up Dbus match for power - determine if host is up and running
     *    or powered off
     * 3. Set up Dbus match for HSBP backplanes and Drives
     * 4. Load HSBP config exposed by entity manager
     *    - Also setup a match to capture HSBP configuation in case
     *      entity-manager restarts
     * 5. Load Clock buffer and IO expander (and other peripherals if any
     *    related to HSBP functionality)
     *    - Reload the info each time HSBP configuration is changed
     * 6. Populate all Backpanes (HSBP's)
     * 7. Load all NVMe drive's and associate with HSBP Backpane
     */

    /* Register GPIO Events on FM_SMB_BMC_NVME_LVC3_ALERT_N */
    if (!hsbpRequestAlertGpioEvents("FM_SMB_BMC_NVME_LVC3_ALERT_N",
                                    nvmeLvc3AlertHandler, nvmeLvc3AlertLine,
                                    nvmeLvc3AlertEvent))
    {
        std::cerr << __FUNCTION__
                  << ": error: Unable to monitor events on HSBP "
                     "Alert line\n";
        return -1;
    }

    /* Setup Dbus-match for power */
    setupPowerMatch(conn);

    /* Setup Dbus-match for HSBP backplanes and Drives */
    setupBackplanesAndDrivesMatch();

    /* Setup HSBP Config match and load config
     * In the event of entity-manager reboot, the match will help catch new
     * configuration.
     * In the event of hsbp-manager reboot, loadHsbpConfig will get all
     * config details and will take care of remaining config's to be
     * loaded
     */
    setupHsbpConfigMatch();
    loadHsbpConfig();

    io.run();
    std::cerr << __FUNCTION__ << ": Aborting hsbp-manager !\n";
    return -1;
}
