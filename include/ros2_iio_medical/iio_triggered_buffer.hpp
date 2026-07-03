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

#ifndef ROS2_IIO_MEDICAL__IIO_TRIGGERED_BUFFER_HPP_
#define ROS2_IIO_MEDICAL__IIO_TRIGGERED_BUFFER_HPP_

// IIO Triggered Buffer Node
//
// Uses Linux IIO kernel buffer + epoll instead of polling sysfs.
// Hardware trigger (e.g. DRDY interrupt from ADS1299) fires the ADC,
// kernel DMA fills the buffer, epoll wakes userspace exactly once per
// sample batch — no busy-wait, no timer jitter.
//
// Sysfs layout used:
//   /sys/bus/iio/devices/iio:deviceN/scan_elements/in_voltageX_en    enable ch
//   /sys/bus/iio/devices/iio:deviceN/scan_elements/in_voltageX_type  format
//   /sys/bus/iio/devices/iio:deviceN/scan_elements/in_voltageX_index order
//   /sys/bus/iio/devices/iio:deviceN/buffer/length                   depth
//   /sys/bus/iio/devices/iio:deviceN/buffer/enable                   start/stop
//   /dev/iio:deviceN                                                  read data

#include "ros2_iio_medical/iio_channel.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <atomic>
#include <string>
#include <thread>
#include <vector>

namespace ros2_iio_medical
{

class IIOTriggeredNode : public rclcpp::Node
{
public:
  explicit IIOTriggeredNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~IIOTriggeredNode();

private:
  // Buffer lifecycle
  bool discover_channels();
  bool enable_channels();
  bool configure_buffer();
  bool open_device();
  void teardown();

  // Acquisition loop — runs in worker thread, woken by epoll
  void read_loop();

  // sysfs helpers
  bool sysfs_write(const std::string & rel_path, const std::string & value);
  bool sysfs_read(const std::string & rel_path, std::string & value);

  // Parameters
  std::string sysfs_path_;   // /sys/bus/iio/devices/iio:device0
  std::string dev_path_;     // /dev/iio:device0
  std::string topic_name_;
  int         num_channels_;
  int         buffer_length_;  // depth in samples

  // Channel layout discovered at runtime
  std::vector<IIOChannelSpec> channels_;
  size_t                      sample_size_ = 0;  // bytes per sample

  // File descriptors
  int iio_fd_    = -1;  // /dev/iio:deviceN
  int epoll_fd_  = -1;
  int event_fd_  = -1;  // eventfd used to unblock epoll on shutdown

  // Worker thread
  std::thread        worker_;
  std::atomic<bool>  running_{false};

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
};

}  // namespace ros2_iio_medical

#endif  // ROS2_IIO_MEDICAL__IIO_TRIGGERED_BUFFER_HPP_
