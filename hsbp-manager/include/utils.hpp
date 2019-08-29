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

#include <cstdint>
#include <string>
#include <variant>
#include <vector>

using GetSubTreeType = std::vector<
    std::pair<std::string,
              std::vector<std::pair<std::string, std::vector<std::string>>>>>;
using BasicVariantType =
    std::variant<std::vector<std::string>, std::string, int64_t, uint64_t,
                 double, int32_t, uint32_t, int16_t, uint16_t, uint8_t, bool>;

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
