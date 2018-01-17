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
#include <cstdint>

struct sensorreadingresp_t {
  uint8_t value;
  uint8_t operation;
  uint8_t indication[2];
} __attribute__((packed));

struct sensorthresholdresp_t {
  uint8_t readable;
  uint8_t lowernc;
  uint8_t lowercritical;
  uint8_t lowernonrecoverable;
  uint8_t uppernc;
  uint8_t uppercritical;
  uint8_t uppernonrecoverable;
} __attribute__((packed));

struct sensoreventenableresp_t {
  uint8_t enabled;
  uint8_t assertionEnabledLSB;
  uint8_t assertionEnabledMSB;
  uint8_t deassertionEnabledLSB;
  uint8_t deassertionEnabledMSB;
} __attribute__((packed));

struct sensoreventstatusresp_t {
  uint8_t enabled;
  uint8_t assertionsLSB;
  uint8_t assertionsMSB;
  // deassertion events currently not supported
  // uint8_t deassertionsLSB;
  // uint8_t deassertionsMSB;
} __attribute__((packed));

enum ipmi_threshold_resp_bits {
  LOWER_NON_CRITICAL,
  LOWER_CRITICAL,
  LOWER_NON_RECOVERABLE,
  UPPER_NON_CRITICAL,
  UPPER_CRITICAL,
  UPPER_NON_RECOVERABLE
};