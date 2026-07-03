cmake_minimum_required(VERSION 3.10...3.27)
project(ros2_iio_medical)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# IIO bridge node (sysfs polling — simple, low rate)
add_executable(iio_bridge src/iio_bridge.cpp)

target_include_directories(iio_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(iio_bridge PUBLIC cxx_std_17)

ament_target_dependencies(iio_bridge
  rclcpp
  std_msgs)

# IIO triggered buffer node (epoll + kernel buffer — hardware-timed, production)
add_executable(iio_triggered_bridge src/iio_triggered_buffer.cpp)

target_include_directories(iio_triggered_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(iio_triggered_bridge PUBLIC cxx_std_17)

ament_target_dependencies(iio_triggered_bridge
  rclcpp
  std_msgs)

# Install
install(TARGETS iio_bridge iio_triggered_bridge
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  # IEC 62304 unit tests — SRS-04 channel parsing, SRS-05 sample extraction
  ament_add_gtest(test_channel_parser test/test_channel_parser.cpp)

  target_include_directories(test_channel_parser PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

  target_compile_features(test_channel_parser PUBLIC cxx_std_17)

  # Suppress strict linters (style enforcers) — not required for this package
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
