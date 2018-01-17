/*
// Copyright (c) 2017 Intel Corporation
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

#pragma once
#include <phosphor-ipmi-host/sensorhandler.h>
#include <cstdint>

static constexpr uint8_t IPMI_SDR_VERSION = 0x51;

struct getsdrinforesp_t {
  uint8_t sdr_version;
  uint8_t record_count_ls;
  uint8_t record_count_ms;
  uint8_t free_space[2];
  uint8_t most_recent_addition[4];
  uint8_t most_recent_erase[4];
  uint8_t opperation_support;
} __attribute__((packed));

struct getsdrresp_t {
  uint8_t record_id_lsb;
  uint8_t record_id_msb;
  get_sdr::SensorDataFullRecord record;
} __attribute__((packed));

enum sensor_type_codes : uint8_t {
  RESERVED = 0x0,
  TEMPERATURE = 0x1,
  VOLTAGE = 0x2,
  CURRENT = 0x3,
  FAN = 0x4,
  OTHER = 0xB,
};

enum sensor_units : uint8_t {
  UNSPECIFIED = 0x0,
  DEGREES_C = 0x1,
  // DEGREEES_F
  // DEGREES_K
  VOLTS = 0x4,
  AMPS = 0x5,
  WATTS = 0x6,
  RPM = 0x12,
};