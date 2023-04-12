// Copyright (c) 2017, Analog Devices Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in
//   the documentation and/or other materials provided with the
//   distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGE.

// #include <string>
// #include "ros/ros.h"
// #include "sensor_msgs/Imu.h"
// #include "sensor_msgs/Temperature.h"
// #include "adi_driver/adis16470.h"
// #include "std_srvs/Trigger.h"

#include "adi_driver2/adis16470_node.hpp"

#include <unistd.h>

using namespace std::chrono_literals;

namespace adi_driver2 {

ImuNode::ImuNode() : Node("adis16470_node") {
  // Read parameters
  declare_parameter("device", "/dev/ttyACM0");
  declare_parameter("frame_id", "imu");
  declare_parameter("burst_mode", true);
  declare_parameter("publish_temperature", true);
  declare_parameter("rate", 100.0);

  device_ = get_parameter("device").as_string();
  frame_id_ = get_parameter("frame_id").as_string();
  burst_mode_ = get_parameter("burst_mode").as_bool();
  publish_temperature_ = get_parameter("publish_temperature").as_bool();
  int loop_rate = 1000 / get_parameter("rate").get_value<double>();
  loop_ms_ = std::chrono::milliseconds{loop_rate};

  RCLCPP_INFO(this->get_logger(), "device: %s", device_.c_str());
  RCLCPP_INFO(this->get_logger(), "frame_id: %s", frame_id_.c_str());
  RCLCPP_INFO(this->get_logger(), "rate: %f [Hz]", loop_rate);
  RCLCPP_INFO(this->get_logger(), "burst_mode: %s",
              (burst_mode_ ? "true" : "false"));
  RCLCPP_INFO(this->get_logger(), "publish_temperature: %s",
              (publish_temperature_ ? "true" : "false"));

  // Data publisher
  imu_data_pub_ = create_publisher<sensor_msgs::msg::Imu>("data_raw", 100);
  if (publish_temperature_) {
    temp_data_pub_ =
        create_publisher<sensor_msgs::msg::Temperature>("temperature", 100);
  }

  // Bias estimate service
  // bias_srv_ = node_handle_.advertiseService("bias_estimate",
  //                                           &ImuNode::bias_estimate, this);
}

ImuNode::~ImuNode() { imu.closePort(); }

/**
 * @brief Check if the device is opened
 */
bool ImuNode::is_opened(void) { return (imu.fd_ >= 0); }
/**
 * @brief Open IMU device file
 */
bool ImuNode::open(void) {
  // Open device file
  if (imu.openPort(device_) < 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open device %s",
                 device_.c_str());
  }
  // Wait 10ms for SPI ready
  usleep(10000);
  int16_t pid = 0;
  imu.get_product_id(pid);
  RCLCPP_INFO(this->get_logger(), "Product ID: %x\n", pid);
  imu.set_bias_estimation_time(0x070a);
}

int ImuNode::publish_imu_data() {
  sensor_msgs::msg::Imu data;
  data.header.frame_id = frame_id_;
  data.header.stamp = rclcpp::Time();

  // Linear acceleration
  data.linear_acceleration.x = imu.accl[0];
  data.linear_acceleration.y = imu.accl[1];
  data.linear_acceleration.z = imu.accl[2];

  // Angular velocity
  data.angular_velocity.x = imu.gyro[0];
  data.angular_velocity.y = imu.gyro[1];
  data.angular_velocity.z = imu.gyro[2];

  // Orientation (not provided)
  data.orientation.x = 0;
  data.orientation.y = 0;
  data.orientation.z = 0;
  data.orientation.w = 1;

  imu_data_pub_->publish(data);
}
int ImuNode::publish_temp_data(void) {
  sensor_msgs::msg::Temperature data;
  data.header.frame_id = frame_id_;
  data.header.stamp = rclcpp::Time();

  // imu Temperature
  data.temperature = imu.temp;
  data.variance = 0;

  temp_data_pub_->publish(data);
}
bool ImuNode::loop() {
  loop_timer_ = create_wall_timer(0.01s, [this]() {
    if (burst_mode_) {
      if (imu.update_burst() == 0) {
        publish_imu_data();
      } else {
        RCLCPP_ERROR(this->get_logger(), "Cannot update burst");
      }
    } else if (publish_temperature_) {
      if (imu.update() == 0) {
        publish_imu_data();
        publish_temp_data();
      } else {
        RCLCPP_ERROR(this->get_logger(), "Cannot update");
      }
    } else if (burst_mode_ && publish_temperature_) {
      if (imu.update_burst() == 0) {
        publish_imu_data();
        publish_temp_data();
      } else {
        RCLCPP_ERROR(this->get_logger(), "Cannot update burst");
      }
    } else {
      if (imu.update() == 0) {
        publish_imu_data();
      } else {
        RCLCPP_ERROR(this->get_logger(), "Cannot update");
      }
    }
  });
  return true;
}
} // namespace adi_driver2

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto ndt_scan_matcher = std::make_shared<adi_driver2::ImuNode>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(ndt_scan_matcher);
  exec.spin();
  rclcpp::shutdown();
  return 0;
  return (0);
}
