#pragma once

#include <boost/algorithm/string/predicate.hpp>
#include <iostream>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

using Association = std::tuple<std::string, std::string, std::string>;

constexpr const char* rootPath = "/xyz/openbmc_project/CallbackManager";
constexpr const char* sensorPath = "/xyz/openbmc_project/sensors";

constexpr const char* globalInventoryIface =
    "xyz.openbmc_project.Inventory.Item.Global";
constexpr const char* associationIface = "org.openbmc.Associations";

namespace threshold
{
constexpr const char* critical = "critical";
constexpr const char* warning = "warning";
} // namespace threshold

struct AssociationManager
{
    AssociationManager(sdbusplus::asio::object_server& objectServer,
                       std::shared_ptr<sdbusplus::asio::connection>& conn) :
        objectServer(objectServer),
        association(objectServer.add_interface(rootPath, associationIface)),
        sensorAssociation(
            objectServer.add_interface(sensorPath, associationIface))
    {
        association->register_property("associations", std::set<Association>());
        sensorAssociation->register_property("associations",
                                             std::set<Association>());
        association->initialize();
        sensorAssociation->initialize();
    }
    ~AssociationManager()
    {
        objectServer.remove_interface(association);
        objectServer.remove_interface(sensorAssociation);
    }

    void setLocalAssociations(const std::vector<std::string>& fatal,
                              const std::vector<std::string>& critical,
                              const std::vector<std::string>& warning)
    {
        std::set<Association> result;

        // fatal maps to redfish critical as refish only has 3 states and LED
        // has 4
        for (const std::string& path : fatal)
        {
            result.emplace(threshold::critical, "", path);
        }
        for (const std::string& path : critical)
        {
            result.emplace(threshold::warning, "", path);
        }
        for (const std::string& path : warning)
        {
            result.emplace(threshold::warning, "", path);
        }
        association->set_property("associations", result);
    }

    void setSensorAssociations(const std::vector<std::string>& critical,
                               const std::vector<std::string>& warning)
    {
        std::set<Association> result;
        for (const std::string& path : critical)
        {
            if (!boost::starts_with(path, sensorPath))
            {
                continue;
            }
            result.emplace(threshold::critical, "", path);
        }
        for (const std::string& path : warning)
        {
            if (!boost::starts_with(path, sensorPath))
            {
                continue;
            }
            result.emplace(threshold::warning, "", path);
        }
        sensorAssociation->set_property("associations", result);
    }

    sdbusplus::asio::object_server& objectServer;
    std::shared_ptr<sdbusplus::asio::dbus_interface> association;
    std::shared_ptr<sdbusplus::asio::dbus_interface> sensorAssociation;
};