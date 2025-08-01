# Callback Manager

Author: James Feist !jfei

Primary assignee: James Feist !jfei

Other contributors: None

Created: 2019-06-26

## Problem Description

We need a centralized location to change the LED state.

## Background and References

[Redfish Health](https://github.com/openbmc/docs/blob/master/designs/redfish_health_rollup.md)

[Sensor Thresholds](https://github.com/openbmc/phosphor-dbus-interfaces/tree/master/xyz/openbmc_project/Sensor/Threshold)

## Requirements

Need to be able to trigger the LED from multiple locations and maintain state.

## Proposed Design

The callback manager can change LED state in the below ways.

1. Monitoring the sensor threshold interface.
1. Monitoring critical / warning thresholds as Redfish does.

Other interfaces specific interfaces can be added later if it makes sense. The
reason it was designed this way is Redfish has the idea of a Global health
state, and a component health state. We need to map these to the LED.

### Thresholds

Warning thresholds will trigger warning (blink green) LED state.

Critical thresholds will trigger critical (blink amber) state.

Callback manager will create associations for redfish for thresholds
automatically.

### Associations

There are two types of status associations, detailed in the Redfish health
whitepaper.

1. Global- These are the attributes to the global state (warning / critical).
   This is exposed by the callback manager for ease
   (xyz.openbmc_project.Inventory.Item.Global,
   path=/xyz/openbmc_project/CallbackManager)
1. Local- Any other warning / critical association.

If the path for an association is Critical for Global & Local, the LED will be
in the fatal (amber solid) state.

If the path for an association is in Warning for Global & Critical for local,
the LED will be in the Critical (amber blink) state.

If the path for an association is in the Warning for Global & not Critical for
local, the LED will be in the Warning (blink green) state.

## Alternatives Considered

Write the LED manually, this seems very error prone.

## Impacts

Will have to add more associations.

## Testing

Sensor override, then look at LED and Redfish.
