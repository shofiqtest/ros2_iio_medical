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

// IIO Triggered Buffer — ROS2 Node
//
// Acquisition path:
//   Hardware DRDY interrupt → IIO trigger → kernel DMA → /dev/iio:deviceN
//   epoll(EPOLLIN) wakes this thread → read() → parse binary → publish ROS2
//
// Compared to sysfs polling (iio_bridge.cpp):
//   ✓ Sample timing driven by hardware interrupt, not software timer
//   ✓ No missed samples between polls
//   ✓ Lower CPU usage — thread sleeps until kernel signals data ready
//   ✓ Correct approach for continuous biosignal acquisition (ECG, EEG)

#include "ros2_iio_medical/iio_triggered_buffer.hpp"
#include "ros2_iio_medical/iio_channel.hpp"

#include <fcntl.h>
#include <sys/epoll.h>
#include <sys/eventfd.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <fstream>
#include <sstream>
#include <stdexcept>

using namespace std::chrono_literals;

namespace ros2_iio_medical
{

// ── Constructor ───────────────────────────────────────────────────────────────

IIOTriggeredNode::IIOTriggeredNode(const rclcpp::NodeOptions & options)
: Node("iio_triggered_bridge", options)
{
  sysfs_path_   = this->declare_parameter<std::string>(
    "sysfs_path", "/sys/bus/iio/devices/iio:device0");
  dev_path_     = this->declare_parameter<std::string>(
    "dev_path", "/dev/iio:device0");
  num_channels_ = this->declare_parameter<int>("num_channels", 8);
  buffer_length_ = this->declare_parameter<int>("buffer_length", 64);
  topic_name_   = this->declare_parameter<std::string>(
    "topic_name", "/biosignal/eeg");

  publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    topic_name_, 10);

  if (!discover_channels()) {
    throw std::runtime_error("Failed to discover IIO channels");
  }
  if (!enable_channels()) {
    throw std::runtime_error("Failed to enable IIO channels");
  }
  if (!configure_buffer()) {
    throw std::runtime_error("Failed to configure IIO buffer");
  }
  if (!open_device()) {
    throw std::runtime_error("Failed to open IIO device");
  }

  running_ = true;
  worker_  = std::thread(&IIOTriggeredNode::read_loop, this);

  RCLCPP_INFO(this->get_logger(),
    "IIO triggered bridge started | device: %s | channels: %zu | "
    "sample_size: %zu bytes | topic: %s",
    dev_path_.c_str(), channels_.size(), sample_size_, topic_name_.c_str());
}

// ── Destructor ────────────────────────────────────────────────────────────────

IIOTriggeredNode::~IIOTriggeredNode()
{
  teardown();
}

// ── Channel discovery ─────────────────────────────────────────────────────────

bool IIOTriggeredNode::discover_channels()
{
  channels_.clear();

  for (int i = 0; i < num_channels_; ++i) {
    IIOChannelSpec spec;
    spec.name  = "in_voltage" + std::to_string(i);

    // index in buffer
    std::string idx_str;
    if (!sysfs_read("scan_elements/" + spec.name + "_index", idx_str)) {
      RCLCPP_WARN(this->get_logger(), "Channel %d has no index — skipping", i);
      continue;
    }
    spec.index = std::stoi(idx_str);

    // type string e.g. "le:s24/32>>0"
    std::string type_str;
    if (!sysfs_read("scan_elements/" + spec.name + "_type", type_str)) {
      RCLCPP_WARN(this->get_logger(), "Channel %d has no type — skipping", i);
      continue;
    }
    if (!ros2_iio_medical::parse_channel_type(type_str, spec)) {
      RCLCPP_WARN(this->get_logger(),
        "Cannot parse type '%s' for channel %d", type_str.c_str(), i);
      continue;
    }

    // scale factor (optional)
    std::string scale_str;
    if (sysfs_read(spec.name + "_scale", scale_str)) {
      spec.scale = std::stod(scale_str);
    } else {
      sysfs_read("in_voltage_scale", scale_str);
      if (!scale_str.empty()) {
        spec.scale = std::stod(scale_str);
      }
    }

    channels_.push_back(spec);
  }

  if (channels_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "No channels discovered");
    return false;
  }

  // Sort by buffer index so extraction offsets are correct
  std::sort(channels_.begin(), channels_.end(),
    [](const IIOChannelSpec & a, const IIOChannelSpec & b) {
      return a.index < b.index;
    });

  // Compute total sample size
  sample_size_ = 0;
  for (const auto & ch : channels_) {
    sample_size_ += ch.storage_bytes();
  }

  RCLCPP_INFO(this->get_logger(),
    "Discovered %zu channels, sample size = %zu bytes",
    channels_.size(), sample_size_);

  return true;
}

// parse_channel_type and extract_sample are free functions in iio_channel.hpp

// ── Enable channels in scan_elements ─────────────────────────────────────────

bool IIOTriggeredNode::enable_channels()
{
  for (int i = 0; i < num_channels_; ++i) {
    std::string path = "scan_elements/in_voltage" + std::to_string(i) + "_en";
    if (!sysfs_write(path, "1")) {
      RCLCPP_WARN(this->get_logger(), "Could not enable channel %d", i);
    }
  }
  return true;
}

// ── Configure kernel buffer ───────────────────────────────────────────────────

bool IIOTriggeredNode::configure_buffer()
{
  // Set buffer depth
  if (!sysfs_write("buffer/length", std::to_string(buffer_length_))) {
    RCLCPP_WARN(this->get_logger(), "Could not set buffer length");
  }

  // Enable buffer — starts DMA acquisition
  if (!sysfs_write("buffer/enable", "1")) {
    RCLCPP_ERROR(this->get_logger(), "Could not enable IIO buffer");
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "IIO buffer enabled, depth = %d", buffer_length_);
  return true;
}

// ── Open /dev/iio:deviceN and set up epoll ────────────────────────────────────

bool IIOTriggeredNode::open_device()
{
  // Open IIO character device non-blocking
  iio_fd_ = ::open(dev_path_.c_str(), O_RDONLY | O_NONBLOCK);
  if (iio_fd_ < 0) {
    RCLCPP_ERROR(this->get_logger(), "open(%s): %s",
      dev_path_.c_str(), std::strerror(errno));
    return false;
  }

  // eventfd for clean shutdown — writing 1 unblocks epoll_wait
  event_fd_ = ::eventfd(0, EFD_NONBLOCK);
  if (event_fd_ < 0) {
    RCLCPP_ERROR(this->get_logger(), "eventfd: %s", std::strerror(errno));
    return false;
  }

  epoll_fd_ = ::epoll_create1(0);
  if (epoll_fd_ < 0) {
    RCLCPP_ERROR(this->get_logger(), "epoll_create1: %s", std::strerror(errno));
    return false;
  }

  // Watch IIO device for incoming data
  struct epoll_event ev_iio {};
  ev_iio.events  = EPOLLIN;
  ev_iio.data.fd = iio_fd_;
  if (::epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, iio_fd_, &ev_iio) < 0) {
    RCLCPP_ERROR(this->get_logger(), "epoll_ctl(iio): %s", std::strerror(errno));
    return false;
  }

  // Watch eventfd for shutdown signal
  struct epoll_event ev_evt {};
  ev_evt.events  = EPOLLIN;
  ev_evt.data.fd = event_fd_;
  if (::epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, event_fd_, &ev_evt) < 0) {
    RCLCPP_ERROR(this->get_logger(), "epoll_ctl(eventfd): %s", std::strerror(errno));
    return false;
  }

  return true;
}

// ── Acquisition loop ──────────────────────────────────────────────────────────

void IIOTriggeredNode::read_loop()
{
  constexpr int MAX_EVENTS = 8;
  struct epoll_event events[MAX_EVENTS];

  std::vector<uint8_t> raw(sample_size_);

  while (running_) {
    int nfds = ::epoll_wait(epoll_fd_, events, MAX_EVENTS, -1 /* block forever */);

    if (nfds < 0) {
      if (errno == EINTR) continue;
      RCLCPP_ERROR(this->get_logger(), "epoll_wait: %s", std::strerror(errno));
      break;
    }

    for (int i = 0; i < nfds; ++i) {
      if (events[i].data.fd == event_fd_) {
        // Shutdown signal received
        return;
      }

      if (events[i].data.fd == iio_fd_) {
        // Read one complete sample from the kernel buffer
        ssize_t bytes = ::read(iio_fd_, raw.data(), sample_size_);

        if (bytes < 0) {
          if (errno == EAGAIN) continue;
          RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "read(iio): %s", std::strerror(errno));
          continue;
        }

        if (static_cast<size_t>(bytes) < sample_size_) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Short read: got %zd, expected %zu", bytes, sample_size_);
          continue;
        }

        // Parse and publish
        auto msg = std_msgs::msg::Float64MultiArray();
        msg.layout.dim.resize(1);
        msg.layout.dim[0].label  = "channels";
        msg.layout.dim[0].size   = static_cast<uint32_t>(channels_.size());
        msg.layout.dim[0].stride = static_cast<uint32_t>(channels_.size());
        msg.data.resize(channels_.size());

        size_t offset = 0;
        for (size_t ch = 0; ch < channels_.size(); ++ch) {
          msg.data[ch] = ros2_iio_medical::extract_sample(raw.data(), offset, channels_[ch]);
          offset += channels_[ch].storage_bytes();
        }

        publisher_->publish(msg);
      }
    }
  }
}


// ── Teardown ──────────────────────────────────────────────────────────────────

void IIOTriggeredNode::teardown()
{
  if (running_.exchange(false)) {
    // Signal epoll to wake up
    if (event_fd_ >= 0) {
      uint64_t val = 1;
      ::write(event_fd_, &val, sizeof(val));
    }
    if (worker_.joinable()) {
      worker_.join();
    }
  }

  // Disable kernel buffer
  sysfs_write("buffer/enable", "0");

  // Disable channels
  for (int i = 0; i < num_channels_; ++i) {
    sysfs_write("scan_elements/in_voltage" + std::to_string(i) + "_en", "0");
  }

  // Close file descriptors
  auto close_fd = [](int & fd) {
    if (fd >= 0) { ::close(fd); fd = -1; }
  };
  close_fd(iio_fd_);
  close_fd(epoll_fd_);
  close_fd(event_fd_);

  RCLCPP_INFO(this->get_logger(), "IIO triggered bridge stopped");
}

// ── sysfs helpers ─────────────────────────────────────────────────────────────

bool IIOTriggeredNode::sysfs_write(
  const std::string & rel_path, const std::string & value)
{
  std::ofstream f(sysfs_path_ + "/" + rel_path);
  if (!f.is_open()) return false;
  f << value;
  return f.good();
}

bool IIOTriggeredNode::sysfs_read(
  const std::string & rel_path, std::string & value)
{
  std::ifstream f(sysfs_path_ + "/" + rel_path);
  if (!f.is_open()) return false;
  std::getline(f, value);
  // Trim whitespace
  value.erase(value.find_last_not_of(" \t\r\n") + 1);
  return !value.empty();
}

}  // namespace ros2_iio_medical

// ── Main ──────────────────────────────────────────────────────────────────────

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ros2_iio_medical::IIOTriggeredNode>());
  rclcpp::shutdown();
  return 0;
}
