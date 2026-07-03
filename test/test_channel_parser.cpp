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

// Unit tests for parse_channel_type() and extract_sample()
//
// IEC 62304 traceability:
//   SRS-04  Channel type parsing   →  TEST_parse_channel_type_*
//   SRS-05  Sample extraction      →  TEST_extract_sample_*

#include <gtest/gtest.h>
#include "ros2_iio_medical/iio_channel.hpp"

using ros2_iio_medical::IIOChannelSpec;
using ros2_iio_medical::parse_channel_type;
using ros2_iio_medical::extract_sample;

// ─────────────────────────────────────────────────────────────────────────────
// parse_channel_type — valid inputs
// ─────────────────────────────────────────────────────────────────────────────

// SRS-04-001: ADS1299 standard type string — le:s24/32>>0
TEST(ParseChannelType, ADS1299_LE_Signed_24_32)
{
  IIOChannelSpec spec;
  ASSERT_TRUE(parse_channel_type("le:s24/32>>0", spec));
  EXPECT_TRUE(spec.little_endian);
  EXPECT_TRUE(spec.is_signed);
  EXPECT_EQ(spec.real_bits, 24u);
  EXPECT_EQ(spec.storage_bits, 32u);
  EXPECT_EQ(spec.shift, 0u);
  EXPECT_EQ(spec.storage_bytes(), 4u);
}

// SRS-04-002: Big-endian unsigned 16-bit (e.g. MAX30102 raw)
TEST(ParseChannelType, BE_Unsigned_16_16)
{
  IIOChannelSpec spec;
  ASSERT_TRUE(parse_channel_type("be:u16/16>>0", spec));
  EXPECT_FALSE(spec.little_endian);
  EXPECT_FALSE(spec.is_signed);
  EXPECT_EQ(spec.real_bits, 16u);
  EXPECT_EQ(spec.storage_bits, 16u);
  EXPECT_EQ(spec.shift, 0u);
  EXPECT_EQ(spec.storage_bytes(), 2u);
}

// SRS-04-003: Non-zero shift (8-bit in 16-bit slot shifted 8)
TEST(ParseChannelType, LE_Signed_8_16_shift8)
{
  IIOChannelSpec spec;
  ASSERT_TRUE(parse_channel_type("le:s8/16>>8", spec));
  EXPECT_TRUE(spec.little_endian);
  EXPECT_TRUE(spec.is_signed);
  EXPECT_EQ(spec.real_bits, 8u);
  EXPECT_EQ(spec.storage_bits, 16u);
  EXPECT_EQ(spec.shift, 8u);
}

// SRS-04-004: 32-bit signed little-endian (ti-ads1298 style)
TEST(ParseChannelType, LE_Signed_32_32)
{
  IIOChannelSpec spec;
  ASSERT_TRUE(parse_channel_type("le:s32/32>>0", spec));
  EXPECT_TRUE(spec.is_signed);
  EXPECT_EQ(spec.real_bits, 32u);
  EXPECT_EQ(spec.storage_bits, 32u);
}

// ─────────────────────────────────────────────────────────────────────────────
// parse_channel_type — invalid inputs
// ─────────────────────────────────────────────────────────────────────────────

// SRS-04-005: Empty string → false
TEST(ParseChannelType, Empty_ReturnsFalse)
{
  IIOChannelSpec spec;
  EXPECT_FALSE(parse_channel_type("", spec));
}

// SRS-04-006: Garbage input → false
TEST(ParseChannelType, Garbage_ReturnsFalse)
{
  IIOChannelSpec spec;
  EXPECT_FALSE(parse_channel_type("not_a_type_string", spec));
}

// SRS-04-007: Missing shift component → false
TEST(ParseChannelType, MissingShift_ReturnsFalse)
{
  IIOChannelSpec spec;
  EXPECT_FALSE(parse_channel_type("le:s24/32", spec));
}

// SRS-04-008: real_bits > storage_bits → invalid, false
TEST(ParseChannelType, RealBitsExceedStorage_ReturnsFalse)
{
  IIOChannelSpec spec;
  EXPECT_FALSE(parse_channel_type("le:s32/16>>0", spec));
}

// ─────────────────────────────────────────────────────────────────────────────
// extract_sample — little-endian signed (ADS1299 style)
// ─────────────────────────────────────────────────────────────────────────────

// SRS-05-001: Positive value, le:s24/32>>0, scale=1.0
TEST(ExtractSample, LE_Signed_24bit_Positive)
{
  IIOChannelSpec spec;
  ASSERT_TRUE(parse_channel_type("le:s24/32>>0", spec));
  spec.scale = 1.0;

  // Value: 0x000100 = 256 stored in 4 bytes LE = {0x00, 0x01, 0x00, 0x00}
  uint8_t buf[] = {0x00, 0x01, 0x00, 0x00};
  EXPECT_DOUBLE_EQ(extract_sample(buf, 0, spec), 256.0);
}

// SRS-05-002: Negative value, le:s24/32>>0 (sign extension)
TEST(ExtractSample, LE_Signed_24bit_Negative)
{
  IIOChannelSpec spec;
  ASSERT_TRUE(parse_channel_type("le:s24/32>>0", spec));
  spec.scale = 1.0;

  // -1 in 24-bit = 0xFFFFFF, stored LE in 32 bits = {0xFF, 0xFF, 0xFF, 0x00}
  uint8_t buf[] = {0xFF, 0xFF, 0xFF, 0x00};
  EXPECT_DOUBLE_EQ(extract_sample(buf, 0, spec), -1.0);
}

// SRS-05-003: Zero value
TEST(ExtractSample, LE_Signed_24bit_Zero)
{
  IIOChannelSpec spec;
  ASSERT_TRUE(parse_channel_type("le:s24/32>>0", spec));
  spec.scale = 1.0;

  uint8_t buf[] = {0x00, 0x00, 0x00, 0x00};
  EXPECT_DOUBLE_EQ(extract_sample(buf, 0, spec), 0.0);
}

// SRS-05-004: Scale factor applied (e.g. 0.000119 mV/LSB for ADS1299)
TEST(ExtractSample, ScaleApplied)
{
  IIOChannelSpec spec;
  ASSERT_TRUE(parse_channel_type("le:s24/32>>0", spec));
  spec.scale = 0.000119;  // ADS1299 at ±4.5V with gain=1

  // Value: 1 LSB → 0.000119 mV
  uint8_t buf[] = {0x01, 0x00, 0x00, 0x00};
  EXPECT_NEAR(extract_sample(buf, 0, spec), 0.000119, 1e-10);
}

// SRS-05-005: Big-endian unsigned 16-bit
TEST(ExtractSample, BE_Unsigned_16bit)
{
  IIOChannelSpec spec;
  ASSERT_TRUE(parse_channel_type("be:u16/16>>0", spec));
  spec.scale = 1.0;

  // Value: 0x0100 = 256, stored BE = {0x01, 0x00}
  uint8_t buf[] = {0x01, 0x00};
  EXPECT_DOUBLE_EQ(extract_sample(buf, 0, spec), 256.0);
}

// SRS-05-006: Buffer offset — second channel starts at byte 4
TEST(ExtractSample, BufferOffset)
{
  IIOChannelSpec spec;
  ASSERT_TRUE(parse_channel_type("le:s24/32>>0", spec));
  spec.scale = 1.0;

  // 8 bytes: ch0 = 0, ch1 = 1
  uint8_t buf[] = {0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00};
  EXPECT_DOUBLE_EQ(extract_sample(buf, 0, spec), 0.0);
  EXPECT_DOUBLE_EQ(extract_sample(buf, 4, spec), 1.0);
}

// SRS-05-007: Non-zero shift
TEST(ExtractSample, NonZeroShift)
{
  IIOChannelSpec spec;
  ASSERT_TRUE(parse_channel_type("le:s8/16>>8", spec));
  spec.scale = 1.0;

  // Value 1 in upper byte: {0x00, 0x01} LE → raw=0x0100 >> 8 = 1
  uint8_t buf[] = {0x00, 0x01};
  EXPECT_DOUBLE_EQ(extract_sample(buf, 0, spec), 1.0);
}

// SRS-05-008: Maximum positive 24-bit signed value (0x7FFFFF = 8388607)
TEST(ExtractSample, MaxPositive_24bit)
{
  IIOChannelSpec spec;
  ASSERT_TRUE(parse_channel_type("le:s24/32>>0", spec));
  spec.scale = 1.0;

  uint8_t buf[] = {0xFF, 0xFF, 0x7F, 0x00};
  EXPECT_DOUBLE_EQ(extract_sample(buf, 0, spec), 8388607.0);
}

// SRS-05-009: Maximum negative 24-bit signed value (0x800000 = -8388608)
TEST(ExtractSample, MaxNegative_24bit)
{
  IIOChannelSpec spec;
  ASSERT_TRUE(parse_channel_type("le:s24/32>>0", spec));
  spec.scale = 1.0;

  uint8_t buf[] = {0x00, 0x00, 0x80, 0x00};
  EXPECT_DOUBLE_EQ(extract_sample(buf, 0, spec), -8388608.0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
