// Copyright 2026 Md Shofiqul Islam
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROS2_IIO_MEDICAL__IIO_CHANNEL_HPP_
#define ROS2_IIO_MEDICAL__IIO_CHANNEL_HPP_

// IIO channel type parsing and sample extraction — pure functions.
//
// Extracted from IIOTriggeredNode so they can be unit-tested without
// instantiating a ROS 2 node or opening hardware file descriptors.
//
// IEC 62304 traceability:
//   SRS-04  parse_channel_type()  →  test/test_channel_parser.cpp
//   SRS-05  extract_sample()      →  test/test_channel_parser.cpp

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

namespace ros2_iio_medical
{

// Describes one IIO channel as parsed from scan_elements sysfs.
struct IIOChannelSpec
{
  int         index         = 0;
  std::string name;
  bool        is_signed     = true;
  uint8_t     real_bits     = 24;   // actual ADC bits (e.g. 24 for ADS1299)
  uint8_t     storage_bits  = 32;   // bits per buffer slot (aligned)
  uint8_t     shift         = 0;
  bool        little_endian = true;
  double      scale         = 1.0;  // LSB → physical unit (mV)

  size_t storage_bytes() const { return static_cast<size_t>(storage_bits) / 8u; }
};

// Parse a Linux IIO type string into an IIOChannelSpec.
//
// Format: "le:s24/32>>0"   (little-endian, signed, 24 real bits, 32 stored, shift 0)
//         "be:u16/16>>0"   (big-endian, unsigned, 16 bits)
//
// Returns true on success, false if the string does not match the format.
inline bool parse_channel_type(const std::string & type_str, IIOChannelSpec & spec)
{
  char endian[4] = {};
  char sign       = 0;
  int  real_bits  = 0;
  int  store_bits = 0;
  int  shift      = 0;

  int matched = std::sscanf(type_str.c_str(),
    "%2[^:]:%c%d/%d>>%d",
    endian, &sign, &real_bits, &store_bits, &shift);

  if (matched != 5) {
    return false;
  }
  if (real_bits <= 0 || store_bits <= 0 || real_bits > store_bits) {
    return false;
  }
  if (sign != 's' && sign != 'u') {
    return false;
  }

  spec.little_endian = (std::string(endian) == "le");
  spec.is_signed     = (sign == 's');
  spec.real_bits     = static_cast<uint8_t>(real_bits);
  spec.storage_bits  = static_cast<uint8_t>(store_bits);
  spec.shift         = static_cast<uint8_t>(shift);

  return true;
}

// Extract one channel sample from a raw binary buffer at byte offset.
//
// Handles:
//   - little-endian and big-endian byte order
//   - signed and unsigned integers
//   - sub-word packing (shift + mask to real_bits)
//   - scale factor → physical unit
inline double extract_sample(
  const uint8_t * buf, size_t offset, const IIOChannelSpec & spec)
{
  uint64_t raw_val = 0;
  const uint8_t * p = buf + offset;
  const uint8_t bytes = static_cast<uint8_t>(spec.storage_bytes());

  if (spec.little_endian) {
    for (uint8_t b = 0; b < bytes; ++b) {
      raw_val |= static_cast<uint64_t>(p[b]) << (8u * b);
    }
  } else {
    for (uint8_t b = 0; b < bytes; ++b) {
      raw_val = (raw_val << 8u) | p[b];
    }
  }

  raw_val >>= spec.shift;

  uint64_t mask = (1ULL << spec.real_bits) - 1ULL;
  raw_val &= mask;

  int64_t signed_val = 0;
  if (spec.is_signed && (raw_val & (1ULL << (spec.real_bits - 1u)))) {
    signed_val = static_cast<int64_t>(raw_val | (~mask));
  } else {
    signed_val = static_cast<int64_t>(raw_val);
  }

  return static_cast<double>(signed_val) * spec.scale;
}

}  // namespace ros2_iio_medical

#endif  // ROS2_IIO_MEDICAL__IIO_CHANNEL_HPP_
