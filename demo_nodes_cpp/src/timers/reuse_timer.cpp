// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#include <chrono>
#include <cstdio>
#include <memory>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class OneOffTimerNode : public rclcpp::Node
{
public:
  OneOffTimerNode()
  : rclcpp::Node("reuse_timer"), count(0)
  {
    one_off_timer = this->create_wall_timer(1s, [this]() {
      printf("in one_off_timer callback\n");
      this->one_off_timer->cancel();
    });
    // cancel immediately to prevent it running the first time.
    one_off_timer->cancel();

    periodic_timer = this->create_wall_timer(2s, [this]() {
      printf("in periodic_timer callback\n");
      if (this->count++ % 3 == 0) {
        printf("  resetting one off timer\n");
        this->one_off_timer->reset();
      } else {
        printf("  not resetting one off timer\n");
      }
    });
  }

  rclcpp::TimerBase::SharedPtr periodic_timer;
  rclcpp::TimerBase::SharedPtr one_off_timer;
  size_t count;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<OneOffTimerNode>();

  rclcpp::spin(node);

  return 0;
}
