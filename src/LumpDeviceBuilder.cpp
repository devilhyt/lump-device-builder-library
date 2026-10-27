// SPDX-FileCopyrightText: 2023-2025 OFDL Robotics Lab
// SPDX-FileCopyrightText: 2023-2025 HsiangYi Tsai <devilhyt@gmail.com>
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "LumpDeviceBuilder.h"

namespace LumpDeviceBuilder::Internal {

  uint32_t versionToBcd(uint32_t version) {
    uint32_t bcd  = 0;
    uint8_t shift = 0;
    while (version) {
      bcd |= (version % 10) << shift;
      version /= 10;
      shift += 4;
    }
    return bcd;
  }

  uint8_t calcChecksum(uint8_t *msg, uint8_t size) {
    uint8_t checksum{0xff};
    while (size--) {
      checksum ^= *msg++;
    }
    return checksum;
  }

  uint8_t queryLog2(uint8_t x) {
    switch (x) {
      case 1:
        return 0;
      case 2:
        return 1;
      case 4:
        return 2;
      case 8:
        return 3;
      case 16:
        return 4;
      case 32:
        return 5;
      default:
        return 255; // error
    }
  }

  uint8_t queryNextPow2(uint8_t x) {
    if (x == 0 || x == 1 || x == 2)
      return x;
    else if (x <= 4)
      return 4;
    else if (x <= 8)
      return 8;
    else if (x <= 16)
      return 16;
    else if (x <= 32)
      return 32;
    else
      return 255; // error
  }

} // namespace LumpDeviceBuilder::Internal
