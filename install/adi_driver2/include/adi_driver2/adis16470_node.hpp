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

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include <string>
// #include "std_srvs/Trigger.h"

#include "adi_driver2/adis16470.h"

namespace adi_driver2 {
class ImuNode : public rclcpp::Node {
public:
  ImuNode();
  ~ImuNode();

  bool is_opened(void);
  bool open(void);
  int publish_imu_data(void);
  int publish_temp_data(void);

  bool loop(void);

  Adis16470 imu;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_data_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_data_pub_;
  //   ros::ServiceServer bias_srv_;

  rclcpp::TimerBase::SharedPtr loop_timer_;

  std::string device_;
  std::string frame_id_;
  bool burst_mode_;
  bool publish_temperature_;
  std::chrono::milliseconds loop_ms_;

  //   bool bias_estimate(std_srvs::Trigger::Request &req,
  //                      std_srvs::Trigger::Response &res) {
  //     ROS_INFO("bias_estimate");
  //     if (imu.bias_correction_update() < 0) {
  //       res.success = false;
  //       res.message = "Bias correction update failed";
  //       return false;
  //     }
  //     res.success = true;
  //     res.message = "Success";
  //     return true;
  //   }
};
} // namespace adi_driver2