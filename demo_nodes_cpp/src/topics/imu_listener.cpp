// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#include <cstdio>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class ImuListener : public rclcpp::Node
{
public:
  ImuListener()
  : Node("imu_listener")
  {
    auto imu_cb =
      [this](const sensor_msgs::msg::Imu::SharedPtr msg) -> void
      {
        RCLCPP_INFO(this->get_logger(), " accel: [%+6.3f %+6.3f %+6.3f]",
          msg->linear_acceleration.x,
          msg->linear_acceleration.y,
          msg->linear_acceleration.z);
      };

    sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "imu", imu_cb, rmw_qos_profile_sensor_data);
  }

private:
  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Imu>> sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImuListener>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
