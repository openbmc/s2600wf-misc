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

#include "callback_manager.hpp"

#include <boost/container/flat_map.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <variant>

constexpr const char* fatalLedPath =
    "/xyz/openbmc_project/led/groups/status_critical";
constexpr const char* criticalLedPath =
    "/xyz/openbmc_project/led/groups/status_non_critical";
constexpr const char* warningLedPath =
    "/xyz/openbmc_project/led/groups/status_degraded";
constexpr const char* okLedPath = "/xyz/openbmc_project/led/groups/status_ok";

constexpr const char* ledIface = "xyz.openbmc_project.Led.Group";
constexpr const char* ledAssertProp = "Asserted";
constexpr const char* ledManagerBusname =
    "xyz.openbmc_project.LED.GroupManager";

std::unique_ptr<AssociationManager> associationManager;

enum class StatusSetting
{
    none,
    ok,
    warn,
    critical,
    fatal
};

constexpr const bool debug = false;

// final led state tracking
StatusSetting currentPriority = StatusSetting::none;

// maps of <object-path, <property, asserted>>
boost::container::flat_map<std::string,
                           boost::container::flat_map<std::string, bool>>
    fatalAssertMap;
boost::container::flat_map<std::string,
                           boost::container::flat_map<std::string, bool>>
    criticalAssertMap;
boost::container::flat_map<std::string,
                           boost::container::flat_map<std::string, bool>>
    warningAssertMap;

std::vector<std::string> assertedInMap(
    const boost::container::flat_map<
        std::string, boost::container::flat_map<std::string, bool>>& map)
{
    std::vector<std::string> ret;
    // if any of the properties are true, return true
    for (const auto& pair : map)
    {
        for (const auto& item : pair.second)
        {
            if (item.second)
            {
                ret.push_back(pair.first);
            }
        }
    }
    return ret;
}

void updateLedStatus(std::shared_ptr<sdbusplus::asio::connection>& conn,
                     bool forceRefresh = false)
{
    std::vector<std::string> fatalVector = assertedInMap(fatalAssertMap);
    bool fatal = fatalVector.size();

    std::vector<std::string> criticalVector = assertedInMap(criticalAssertMap);
    bool critical = criticalVector.size();

    std::vector<std::string> warningVector = assertedInMap(warningAssertMap);
    bool warn = warningVector.size();

    associationManager->setLocalAssociations(fatalVector, criticalVector,
                                             warningVector);

    StatusSetting last = currentPriority;

    if (fatal)
    {
        currentPriority = StatusSetting::fatal;
    }
    else if (critical)
    {
        currentPriority = StatusSetting::critical;
    }
    else if (warn)
    {
        currentPriority = StatusSetting::warn;
    }
    else
    {
        currentPriority = StatusSetting::ok;
    }

    std::vector<std::pair<std::string, std::variant<bool>>> ledsToSet;

    if (last != currentPriority || forceRefresh)
    {
        switch (currentPriority)
        {
            case (StatusSetting::fatal):
            {
                ledsToSet.push_back(std::make_pair(fatalLedPath, true));
                ledsToSet.push_back(std::make_pair(criticalLedPath, false));
                ledsToSet.push_back(std::make_pair(warningLedPath, false));
                ledsToSet.push_back(std::make_pair(okLedPath, false));
                break;
            }
            case (StatusSetting::critical):
            {
                ledsToSet.push_back(std::make_pair(fatalLedPath, false));
                ledsToSet.push_back(std::make_pair(criticalLedPath, true));
                ledsToSet.push_back(std::make_pair(warningLedPath, false));
                ledsToSet.push_back(std::make_pair(okLedPath, false));
                break;
            }
            case (StatusSetting::warn):
            {
                ledsToSet.push_back(std::make_pair(fatalLedPath, false));
                ledsToSet.push_back(std::make_pair(criticalLedPath, false));
                ledsToSet.push_back(std::make_pair(warningLedPath, true));
                ledsToSet.push_back(std::make_pair(okLedPath, false));
                break;
            }
            case (StatusSetting::ok):
            {
                ledsToSet.push_back(std::make_pair(fatalLedPath, false));
                ledsToSet.push_back(std::make_pair(criticalLedPath, false));
                ledsToSet.push_back(std::make_pair(warningLedPath, false));
                ledsToSet.push_back(std::make_pair(okLedPath, true));
                break;
            }
        }
    }

    for (const auto& ledPair : ledsToSet)
    {
        conn->async_method_call(
            [ledPair](const boost::system::error_code ec) {
                if (ec)
                {
                    std::cerr << "Cannot set " << ledPair.first << " to "
                              << std::boolalpha
                              << std::get<bool>(ledPair.second) << "\n";
                }
                if constexpr (debug)
                {
                    std::cerr << "Set " << ledPair.first << " to "
                              << std::boolalpha
                              << std::get<bool>(ledPair.second) << "\n";
                }
            },
            ledManagerBusname, ledPair.first, "org.freedesktop.DBus.Properties",
            "Set", ledIface, ledAssertProp, ledPair.second);
    }
}

void createThresholdMatch(std::shared_ptr<sdbusplus::asio::connection>& conn)
{

    static sdbusplus::bus::match::match match(
        static_cast<sdbusplus::bus::bus&>(*conn),
        "type='signal',interface='org.freedesktop.DBus.Properties',"
        "path_"
        "namespace='/xyz/openbmc_project/"
        "sensors',arg0namespace='xyz.openbmc_project.Sensor.Threshold'",
        [&conn](sdbusplus::message::message& message) {
            std::string objectName;
            boost::container::flat_map<std::string, std::variant<bool>> values;

            try
            {
                message.read(objectName, values);
            }
            catch (sdbusplus::exception_t&)
            {
                return;
            }
            if constexpr (debug)
            {
                std::cerr << "Threshold callback " << message.get_path()
                          << "\n";
            }

            auto findCriticalLow = values.find("CriticalAlarmLow");
            auto findCriticalHigh = values.find("CriticalAlarmHigh");

            auto findWarnLow = values.find("WarningAlarmLow");
            auto findWarnHigh = values.find("WarningAlarmHigh");

            if (findCriticalLow != values.end())
            {
                criticalAssertMap[message.get_path()]["Low"] =
                    std::get<bool>(findCriticalLow->second);
            }
            if (findCriticalHigh != values.end())
            {
                criticalAssertMap[message.get_path()]["High"] =
                    std::get<bool>(findCriticalHigh->second);
            }
            if (findWarnLow != values.end())
            {
                warningAssertMap[message.get_path()]["Low"] =
                    std::get<bool>(findWarnLow->second);
            }
            if (findWarnHigh != values.end())
            {
                warningAssertMap[message.get_path()]["High"] =
                    std::get<bool>(findWarnHigh->second);
            }

            associationManager->setSensorAssociations(
                assertedInMap(criticalAssertMap),
                assertedInMap(warningAssertMap));

            updateLedStatus(conn);
        });
}

void createAssociationMatch(std::shared_ptr<sdbusplus::asio::connection>& conn)
{
    static sdbusplus::bus::match::match match(
        static_cast<sdbusplus::bus::bus&>(*conn),
        "type='signal',interface='org.freedesktop.DBus.Properties',"
        "arg0namespace='" +
            std::string(associationIface) + "'",
        [&conn](sdbusplus::message::message& message) {
            if (message.get_path() == rootPath)
            {
                return; // it's us
            }
            std::string objectName;
            boost::container::flat_map<std::string,
                                       std::variant<std::vector<Association>>>
                values;
            try
            {
                message.read(objectName, values);
            }
            catch (sdbusplus::exception_t&)
            {
                return;
            }

            if constexpr (debug)
            {
                std::cerr << "Association callback " << message.get_path()
                          << "\n";
            }

            auto findAssociations = values.find("associations");
            if (findAssociations == values.end())
            {
                return;
            }
            const std::vector<Association>* associations =
                std::get_if<std::vector<Association>>(
                    &findAssociations->second);

            if (associations == nullptr)
            {
                std::cerr << "Illegal Association on " << message.get_path()
                          << "\n";
                return;
            }

            bool localWarning = false;
            bool localCritical = false;
            bool globalWarning = false;
            bool globalCritical = false;

            for (const auto& [forward, reverse, path] : *associations)
            {
                if (path == rootPath)
                {
                    globalWarning = globalWarning ? true : reverse == "warning";
                    globalCritical =
                        globalCritical ? true : reverse == "critical";

                    if constexpr (1)
                    {
                        std::cerr << "got global ";
                    }
                }
                else
                {
                    localWarning = localWarning ? true : reverse == "warning";
                    localCritical =
                        localCritical ? true : reverse == "critical";
                }
                if (globalCritical && localCritical)
                {
                    break;
                }
            }

            bool fatal = globalCritical && localCritical;
            bool critical = globalWarning && localCritical;
            bool warning = globalWarning && localWarning;

            fatalAssertMap[message.get_path()]["association"] = fatal;
            criticalAssertMap[message.get_path()]["association"] = critical;
            warningAssertMap[message.get_path()]["association"] = warning;

            updateLedStatus(conn);
        });
}

int main(int argc, char** argv)
{
    boost::asio::io_service io;
    auto conn = std::make_shared<sdbusplus::asio::connection>(io);
    conn->request_name("xyz.openbmc_project.CallbackManager");
    sdbusplus::asio::object_server objServer(conn);
    std::shared_ptr<sdbusplus::asio::dbus_interface> rootIface =
        objServer.add_interface(rootPath,
                                "xyz.openbmc_project.CallbackManager");
    rootIface->register_method("RetriggerLEDUpdate",
                               [&conn]() { updateLedStatus(conn, true); });
    rootIface->initialize();

    std::shared_ptr<sdbusplus::asio::dbus_interface> inventoryIface =
        objServer.add_interface(rootPath, globalInventoryIface);
    inventoryIface->initialize();

    associationManager = std::make_unique<AssociationManager>(objServer, conn);

    createThresholdMatch(conn);
    createAssociationMatch(conn);
    updateLedStatus(conn);

    io.run();

    return 0;
}
