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

#include <boost/algorithm/string/predicate.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/container/flat_map.hpp>
#include <cstdint>
#include <iostream>
#include <sdbusplus/asio/connection.hpp>
#include <string>
#include <variant>
#include <vector>

using GetSubTreeType = std::vector<
    std::pair<std::string,
              std::vector<std::pair<std::string, std::vector<std::string>>>>>;
using BasicVariantType =
    std::variant<std::vector<std::string>, std::string, int64_t, uint64_t,
                 double, int32_t, uint32_t, int16_t, uint16_t, uint8_t, bool>;
using Association = std::tuple<std::string, std::string, std::string>;

namespace mapper
{
constexpr const char* busName = "xyz.openbmc_project.ObjectMapper";
constexpr const char* path = "/xyz/openbmc_project/object_mapper";
constexpr const char* interface = "xyz.openbmc_project.ObjectMapper";
constexpr const char* subtree = "GetSubTree";
} // namespace mapper

namespace entityManager
{
constexpr const char* busName = "xyz.openbmc_project.EntityManager";
} // namespace entityManager

namespace inventory
{
constexpr const char* interface = "xyz.openbmc_project.Inventory.Item";
} // namespace inventory

namespace ledGroup
{
constexpr const char* interface = "xyz.openbmc_project.Led.Group";
constexpr const char* asserted = "Asserted";
} // namespace ledGroup

namespace properties
{
constexpr const char* interface = "org.freedesktop.DBus.Properties";
constexpr const char* get = "Get";
} // namespace properties

namespace power
{
const static constexpr char* busname = "xyz.openbmc_project.State.Host";
const static constexpr char* interface = "xyz.openbmc_project.State.Host";
const static constexpr char* path = "/xyz/openbmc_project/state/host0";
const static constexpr char* property = "CurrentHostState";
} // namespace power

namespace hsbp
{
enum class registers : uint8_t
{
    fpgaIdH = 0x0,
    fpgaIdL = 0x1,
    typeId = 0x2,
    bootVer = 0x3,
    fpgaVer = 0x4,
    securityRev = 0x5,
    funSupported = 0x6,
    numDisks = 0x7,
    presence = 0x8,
    ssdIFDET = 0x9,
    ifdetPart = 0xA,
    statusLocate = 0xB,
    statusFail = 0xC,
    statusRebuild = 0xD,
    ledOverride = 0xE,
    ledStatus = 0xF,
    ledPattern0 = 0x10,
    ledPattern1 = 0x11,
    ledPattern2 = 0x12,
    ledPattern3 = 0x13,
    ledPattern4 = 0x14,
    ledPattern5 = 0x15,
    ledPattern6 = 0x16,
    ledPattern7 = 0x17,
};

} // namespace hsbp

static std::unique_ptr<sdbusplus::bus::match::match> powerMatch = nullptr;
static bool powerStatusOn = false;

bool isPowerOn(void)
{
    if (!powerMatch)
    {
        throw std::runtime_error("Power Match Not Created");
    }
    return powerStatusOn;
}

void setupPowerMatch(const std::shared_ptr<sdbusplus::asio::connection>& conn)
{
    static boost::asio::steady_timer timer(conn->get_io_context());
    // create a match for powergood changes, first time do a method call to
    // cache the correct value
    if (powerMatch)
    {
        return;
    }

    powerMatch = std::make_unique<sdbusplus::bus::match::match>(
        static_cast<sdbusplus::bus::bus&>(*conn),
        "type='signal',interface='" + std::string(properties::interface) +
            "',path='" + std::string(power::path) + "',arg0='" +
            std::string(power::interface) + "'",
        [](sdbusplus::message::message& message) {
            std::string objectName;
            boost::container::flat_map<std::string, std::variant<std::string>>
                values;
            message.read(objectName, values);
            auto findState = values.find(power::property);
            if (findState != values.end())
            {
                bool on = boost::ends_with(
                    std::get<std::string>(findState->second), "Running");
                if (!on)
                {
                    timer.cancel();
                    powerStatusOn = false;
                    return;
                }
                // on comes too quickly
                timer.expires_after(std::chrono::seconds(10));
                timer.async_wait([](boost::system::error_code ec) {
                    if (ec == boost::asio::error::operation_aborted)
                    {
                        return;
                    }
                    else if (ec)
                    {
                        std::cerr << "Timer error " << ec.message() << "\n";
                        return;
                    }
                    powerStatusOn = true;
                });
            }
        });

    conn->async_method_call(
        [](boost::system::error_code ec,
           const std::variant<std::string>& state) {
            if (ec)
            {
                // we commonly come up before power control, we'll capture the
                // property change later
                return;
            }
            powerStatusOn =
                boost::ends_with(std::get<std::string>(state), "Running");
        },
        power::busname, power::path, properties::interface, properties::get,
        power::interface, power::property);
}
