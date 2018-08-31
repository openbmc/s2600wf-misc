/*
// Copyright (c) 2018 Intel Corporation
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
#include <systemd/sd-journal.h>

#include <boost/container/flat_map.hpp>
#include <boost/container/flat_set.hpp>
#include <experimental/string_view>
#include <iostream>
#include <ipmi/sensorutils.hpp>
#include <sdbusplus/asio/object_server.hpp>

static constexpr char const *ipmiSelObject = "xyz.openbmc_project.Logging.IPMI";
static constexpr char const *ipmiSelPath = "/xyz/openbmc_project/Logging/IPMI";
static constexpr char const *ipmiSelAddInterface =
    "xyz.openbmc_project.Logging.IPMI";

// ID string generated using journalctl to include in the MESSAGE_ID field for
// SEL entries.  Helps with filtering SEL entries in the journal.
static constexpr char const *selMessageId = "b370836ccf2f4850ac5bee185b77893a";
static constexpr int selPriority = 5; // notice
static constexpr uint8_t selSystemType = 0x02;
static constexpr uint16_t selBmcGenId = 0x0020;
static constexpr uint16_t selInvalidRecID = 0xFFFF;
static constexpr size_t selEvtDataMaxSize = 3;
static constexpr size_t selOemDataMaxSize = 13;

typedef struct
{
    uint8_t recordType;
    uint8_t genId;
    bool assert;
    std::string path;
    std::vector<uint8_t> data;
} SelInfo;

struct DBusInternalError final : public sdbusplus::exception_t
{
    const char *name() const noexcept override
    {
        return "org.freedesktop.DBus.Error.Failed";
    };
    const char *description() const noexcept override
    {
        return "internal error";
    };
    const char *what() const noexcept override
    {
        return "org.freedesktop.DBus.Error.Failed: "
               "internal error";
    };
};

struct VariantToDoubleVisitor
{
    template <typename T>
    std::enable_if_t<std::is_arithmetic<T>::value, double>
        operator()(const T &t) const
    {
        return static_cast<double>(t);
    }
    template <typename T>
    std::enable_if_t<!std::is_arithmetic<T>::value, double>
        operator()(const T &t) const
    {
        throw std::invalid_argument("Cannot translate type to double");
    }
};

static unsigned int initializeRecordId(void)
{
    int ret;
    sd_journal *journal;
    ret = sd_journal_open(&journal, SD_JOURNAL_LOCAL_ONLY);
    if (ret < 0)
    {
        std::cerr << "Failed to open journal: " << strerror(-ret) << "\n";
        throw DBusInternalError();
    }
    unsigned int recordId = selInvalidRecID;
    char match[256] = {};
    snprintf(match, sizeof(match), "MESSAGE_ID=%s", selMessageId);
    sd_journal_add_match(journal, match, 0);
    SD_JOURNAL_FOREACH_BACKWARDS(journal)
    {
        const char *data = nullptr;
        size_t length;

        ret = sd_journal_get_data(journal, "IPMI_SEL_RECORD_ID",
                                  (const void **)&data, &length);
        if (ret < 0)
        {
            std::cerr << "Failed to read IPMI_SEL_RECORD_ID field: "
                      << strerror(-ret) << "\n";
            continue;
        }
        if (ret = sscanf(data, "IPMI_SEL_RECORD_ID=%u", &recordId) != 1)
        {
            std::cerr << "Failed to parse record ID: " << ret << "\n";
            throw DBusInternalError();
        }
        break;
    }
    sd_journal_close(journal);
    return recordId;
}

static unsigned int getNewRecordId(void)
{
    static unsigned int recordId = initializeRecordId();

    if (++recordId >= selInvalidRecID)
    {
        recordId = 1;
    }
    return recordId;
}

static void toHexStr(const std::vector<uint8_t> &data, char *hexStr,
                     const size_t &hexStrSize)
{
    if (data.size() * 2 + 1 > hexStrSize)
    {
        return;
    }
    char *ptr = hexStr;
    for (auto &v : data)
    {
        ptr += snprintf(ptr, (hexStr + hexStrSize) - ptr, "%02X", v);
    }
}

static uint16_t selAddSystemRecord(const std::string &message,
                                   const std::string &path,
                                   const std::vector<uint8_t> &selData,
                                   const bool &assert,
                                   const uint16_t &genId = selBmcGenId)
{
    // Only 3 bytes of SEL event data are allowed in a system record
    if (selData.size() > selEvtDataMaxSize)
    {
        throw std::invalid_argument("Event data too large");
    }
    char selDataStr[selData.size() * 2 + 1];
    toHexStr(selData, selDataStr, sizeof(selDataStr));

    unsigned int recordId = getNewRecordId();
    sd_journal_send(
        "MESSAGE=%s", message.c_str(), "PRIORITY=%i", selPriority,
        "MESSAGE_ID=%s", selMessageId, "IPMI_SEL_RECORD_ID=%d", recordId,
        "IPMI_SEL_RECORD_TYPE=%x", selSystemType, "IPMI_SEL_GENERATOR_ID=%x",
        genId, "IPMI_SEL_SENSOR_PATH=%s", path.c_str(), "IPMI_SEL_EVENT_DIR=%x",
        assert, "IPMI_SEL_DATA=%s", selDataStr, NULL);
    return recordId;
}

static uint16_t selAddOemRecord(const std::string &message,
                                const std::vector<uint8_t> &selData,
                                const uint8_t &recordType)
{
    // A maximum of 13 bytes of SEL event data are allowed in an OEM record
    if (selData.size() > selOemDataMaxSize)
    {
        throw std::invalid_argument("Event data too large");
    }
    char selDataStr[selData.size() * 2 + 1];
    toHexStr(selData, selDataStr, sizeof(selDataStr));

    unsigned int recordId = getNewRecordId();
    sd_journal_send("MESSAGE=%s", message.c_str(), "PRIORITY=%i", selPriority,
                    "MESSAGE_ID=%s", selMessageId, "IPMI_SEL_RECORD_ID=%d",
                    recordId, "IPMI_SEL_RECORD_TYPE=%x", recordType,
                    "IPMI_SEL_DATA=%s", selDataStr, NULL);
    return recordId;
}

enum thresholdEventOffsets
{
    lowerNonCritGoingLow = 0x00,
    lowerCritGoingLow = 0x02,
    upperNonCritGoingHigh = 0x07,
    upperCritGoingHigh = 0x09,
};

static sdbusplus::bus::match::match startThresholdEventMonitor(
    std::shared_ptr<sdbusplus::asio::connection> conn)
{
    auto thresholdEventMatcherCallback = [conn](
                                             sdbusplus::message::message &msg) {
        // This static set of std::pair<path, event> tracks asserted events to
        // avoid duplicate logs or deasserts logged without an assert
        static boost::container::flat_set<std::pair<std::string, std::string>>
            assertedEvents;
        // SEL event data is three bytes where 0xFF means unspecified
        std::vector<uint8_t> eventData(selEvtDataMaxSize, 0xFF);

        // Get the event type and assertion details from the message
        std::string thresholdInterface;
        boost::container::flat_map<std::string,
                                   sdbusplus::message::variant<bool>>
            propertiesChanged;
        msg.read(thresholdInterface, propertiesChanged);
        std::string event = propertiesChanged.begin()->first;
        if (!propertiesChanged.begin()->second.is<bool>())
        {
            std::cerr << "threshold event direction has invalid type\n";
            return;
        }
        bool assert = propertiesChanged.begin()->second.get_unchecked<bool>();

        // Check the asserted events to determine if we should log this event
        std::pair<std::string, std::string> pathAndEvent(
            std::string(msg.get_path()), event);
        if (assert)
        {
            // For asserts, add the event to the set and only log it if it's new
            if (assertedEvents.insert(pathAndEvent).second == false)
            {
                // event is already in the set
                return;
            }
        }
        else
        {
            // For deasserts, remove the event and only log the deassert if it
            // was asserted
            if (assertedEvents.erase(pathAndEvent) == 0)
            {
                // asserted event was not in the set
                return;
            }
        }

        // Set the IPMI threshold event type based on the event details from the
        // message
        if (event == "CriticalAlarmLow")
        {
            eventData[0] = lowerCritGoingLow;
        }
        else if (event == "WarningAlarmLow")
        {
            eventData[0] = lowerNonCritGoingLow;
        }
        else if (event == "WarningAlarmHigh")
        {
            eventData[0] = upperNonCritGoingHigh;
        }
        else if (event == "CriticalAlarmHigh")
        {
            eventData[0] = upperCritGoingHigh;
        }
        // Indicate that bytes 2 and 3 are threshold sensor trigger values by
        // setting bits 6 and 4
        eventData[0] |= 0x50;

        // Get the sensor reading to put in the event data
        sdbusplus::message::message getSensorValue =
            conn->new_method_call(msg.get_sender(), msg.get_path(),
                                  "org.freedesktop.DBus.Properties", "GetAll");
        getSensorValue.append("xyz.openbmc_project.Sensor.Value");
        boost::container::flat_map<std::string,
                                   sdbusplus::message::variant<double>>
            sensorValue;
        try
        {
            sdbusplus::message::message getSensorValueResp =
                conn->call(getSensorValue);
            getSensorValueResp.read(sensorValue);
        }
        catch (sdbusplus::exception_t &)
        {
            std::cerr << "error getting sensor value from " << msg.get_path()
                      << "\n";
            return;
        }
        double max = mapbox::util::apply_visitor(VariantToDoubleVisitor(),
                                                 sensorValue["MaxValue"]);
        double min = mapbox::util::apply_visitor(VariantToDoubleVisitor(),
                                                 sensorValue["MinValue"]);
        double sensorVal = mapbox::util::apply_visitor(VariantToDoubleVisitor(),
                                                       sensorValue["Value"]);
        eventData[1] = ipmi::getScaledIPMIValue(sensorVal, max, min);

        // Get the threshold value to put in the event data
        // Get the threshold parameter by removing the "Alarm" text from the
        // event string
        std::string alarm("Alarm");
        std::string::size_type pos = event.find(alarm);
        if (pos != std::string::npos)
        {
            event.erase(pos, alarm.length());
        }
        sdbusplus::message::message getThreshold =
            conn->new_method_call(msg.get_sender(), msg.get_path(),
                                  "org.freedesktop.DBus.Properties", "Get");
        getThreshold.append(thresholdInterface, event);
        sdbusplus::message::variant<double> thresholdValue;
        try
        {
            sdbusplus::message::message getThresholdResp =
                conn->call(getThreshold);
            getThresholdResp.read(thresholdValue);
        }
        catch (sdbusplus::exception_t &)
        {
            std::cerr << "error getting sensor threshold from "
                      << msg.get_path() << "\n";
            return;
        }
        double thresholdVal = mapbox::util::apply_visitor(
            VariantToDoubleVisitor(), thresholdValue);
        eventData[2] = ipmi::getScaledIPMIValue(thresholdVal, max, min);

        // Construct a human-readable message of this event for the log
        std::experimental::string_view sensorName(msg.get_path());
        sensorName.remove_prefix(
            std::min(sensorName.find_last_of("/") + 1, sensorName.size()));
        std::string journalMsg(sensorName.to_string() +
                               (assert ? " asserted " : " deasserted ") +
                               propertiesChanged.begin()->first +
                               ". Reading=" + std::to_string(sensorVal) +
                               " Threshold=" + std::to_string(thresholdVal));

        selAddSystemRecord(journalMsg, std::string(msg.get_path()), eventData,
                           assert);
    };
    sdbusplus::bus::match::match thresholdEventMatcher(
        static_cast<sdbusplus::bus::bus &>(*conn),
        "type='signal',interface='org.freedesktop.DBus.Properties',member='"
        "PropertiesChanged',arg0namespace='xyz.openbmc_project.Sensor."
        "Threshold'",
        std::move(thresholdEventMatcherCallback));
    return thresholdEventMatcher;
}

int main(int argc, char *argv[])
{
    // setup connection to dbus
    boost::asio::io_service io;
    auto conn = std::make_shared<sdbusplus::asio::connection>(io);

    // IPMI SEL Object
    conn->request_name(ipmiSelObject);
    auto server = sdbusplus::asio::object_server(conn);

    // Add SEL Interface
    std::shared_ptr<sdbusplus::asio::dbus_interface> ifaceAddSel =
        server.add_interface(ipmiSelPath, ipmiSelAddInterface);

    // Add a new SEL entry
    ifaceAddSel->register_method(
        "IpmiSelAdd", [](const std::string &message, const std::string &path,
                         const std::vector<uint8_t> &selData,
                         const bool &assert, const uint16_t &genId) {
            return selAddSystemRecord(message, path, selData, assert, genId);
        });
    // Add a new OEM SEL entry
    ifaceAddSel->register_method(
        "IpmiSelAddOem",
        [](const std::string &message, const std::vector<uint8_t> &selData,
           const uint8_t &recordType) {
            return selAddOemRecord(message, selData, recordType);
        });
    ifaceAddSel->initialize();

    sdbusplus::bus::match::match thresholdEventMonitor =
        startThresholdEventMonitor(conn);

    io.run();

    return 0;
}
