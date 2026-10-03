#include <cstdint>
#include <cstdlib>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/string.hpp"

namespace demo_nodes_cpp
{

// elevarrrr al cuadrado el número recibido
class NumberProcessor : public rclcpp::Node
{
public:
  explicit NumberProcessor(const rclcpp::NodeOptions & options)
  : Node("number_processor", options)
  {
    pub_ = this->create_publisher<std_msgs::msg::String>("processed_data", 10);

    sub_ = this->create_subscription<std_msgs::msg::String>(
      "chatter", 10,
      [this](const std_msgs::msg::String & msg) -> void
      {
  
        std::string number_text = msg.data;
        size_t pos = number_text.find(':');
        if (pos != std::string::npos) {
          number_text = number_text.substr(pos + 1);
        }

        uint64_t n = std::strtoull(number_text.c_str(), nullptr, 10);
        uint64_t result = n * n;

        RCLCPP_INFO(
          this->get_logger(), "%s² = %s",
          std::to_string(n).c_str(), std::to_string(result).c_str());
     
        std_msgs::msg::String out;
        out.data = "Result: " + std::to_string(result);
        pub_->publish(out);
      });
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

}  // namespace demo_nodes_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(demo_nodes_cpp::NumberProcessor)