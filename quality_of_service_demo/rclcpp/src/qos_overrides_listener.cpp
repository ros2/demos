// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/qos_overriding_options.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "sensor_msgs/msg/image.hpp"

#include "quality_of_service_demo/visibility_control.h"

namespace quality_of_service_demo
{
class QosOverridesListener : public rclcpp::Node
{
public:
  QUALITY_OF_SERVICE_DEMO_PUBLIC
  explicit QosOverridesListener(const rclcpp::NodeOptions & options)
  : Node("qos_overrides_listener", options)
  {
    // Callback that will be used when a message is received.
    auto callback =
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) -> void
      {
        rclcpp::Time now = this->get_clock()->now();
        auto diff = now - msg->header.stamp;
        RCLCPP_INFO(
          this->get_logger(),
          "I heard an image. Message single trip latency: [%f]",
          diff.seconds());
      };
    rclcpp::SubscriptionOptions sub_opts;
    // Update the subscription options to allow reconfigurable qos settings.
    sub_opts.qos_overriding_options = rclcpp::QosOverridingOptions {
      {
        // Here all policies that are desired to be reconfigurable are listed.
        rclcpp::QosPolicyKind::Depth,
        rclcpp::QosPolicyKind::Durability,
        rclcpp::QosPolicyKind::History,
        rclcpp::QosPolicyKind::Reliability,
      },
      [](const rclcpp::QoS & qos) {
        /** This is a qos validation callback, that can optionally be provided.
         * Here, arbitrary constraints in the final qos profile can be checked.
         * The function will return true if the user provided qos profile is accepted.
         * If the profile is not accepted, the user will get an InvalidQosOverridesException.
         */
        rclcpp::QosCallbackResult result;
        result.successful = false;
        if (qos.depth() > 10u) {
          result.reason = "expected history depth less or equal than 10";
          return result;
        }
        result.successful = true;
        return result;
      }
      /**
       * The "id" option is useful when you want to have subscriptions
       * listening to the same topic with different QoS profiles within a node.
       * You can try uncommenting and modifying
       * `qos_overrides./qos_overrides_topic.subscription`
       * to
       * `qos_overrides./qos_overrides_topic.subscription_custom_identifier`
       *
       * Uncomment the next line to try it.
       */
      // , "custom_identifier"
    };
    // create the subscription
    sub_ = create_subscription<sensor_msgs::msg::Image>(
      "qos_overrides_chatter", 1, callback, sub_opts);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

}  // namespace quality_of_service_demo

RCLCPP_COMPONENTS_REGISTER_NODE(quality_of_service_demo::QosOverridesListener)
