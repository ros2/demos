//
// Created by deepanshu on 6/1/22.
//

#include "string"
#include "memory"

#include <utility>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logging.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

// Notes: // failure in one param does not affect second param. --> validation

class ParamTest{
 public:
  explicit ParamTest(rclcpp::Node::SharedPtr node);

 private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_ = rclcpp::get_logger("### ParamTest");

  // validation handles
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_;

  // post validate handles
  rclcpp::node_interfaces::OnSetLocalParametersCallbackHandle::SharedPtr local_callback_handle1_;

  // validation
  rcl_interfaces::msg::SetParametersResult parametersCallback(
      const std::vector<rclcpp::Parameter> &parameters);

  // post process
  void localParametersCallback(const std::vector<rclcpp::Parameter> &parameters);
};

ParamTest::ParamTest(rclcpp::Node::SharedPtr node) : node_(std::move(node)) {
    RCLCPP_WARN_STREAM(this->logger_, "ParamTest");

    // see what happens when you declare a param
    node_->declare_parameter<double>("param1", 1.0);
    node_->declare_parameter<double>("param2", 2.0);

    double value1 = node_->get_parameter("param1").as_double();
    double value2 = node_->get_parameter("param2").as_double();

    // validation callbacks
    callback_handle_ = node_->add_on_set_parameters_callback(
        std::bind(&ParamTest::parametersCallback, this, std::placeholders::_1));

    // post set callbacks for local support
    local_callback_handle1_ = node_->add_local_parameters_callback(
      std::bind(&ParamTest::localParametersCallback, this, std::placeholders::_1));
}

// failure in one param does not affect second param.
rcl_interfaces::msg::SetParametersResult ParamTest::parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  RCLCPP_WARN_STREAM(this->logger_, "validation --> parametersCallback");
  for(const auto&param:parameters){
    std::cout << "###" << param.get_name() << std::endl;
    if(param.get_name() == "param1"){
      RCLCPP_ERROR_STREAM(this->logger_, "### Received param1");
      result.successful = true;
      result.reason = "success param1";
    }
    if(param.get_name() == "param2"){
      RCLCPP_ERROR_STREAM(this->logger_, "### Received param2");
      result.successful = true;
      result.reason = "success param2";
    }
  }

  // Here update class attributes, do some actions, etc.
  return result;
}

void ParamTest::localParametersCallback(const std::vector<rclcpp::Parameter> &parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  RCLCPP_ERROR_STREAM(this->logger_, "post --> localParametersCallback");
  for(const auto&param:parameters){
    std::cout << "###" << param.get_name() << std::endl;
    if(param.get_name() == "param1"){
      RCLCPP_ERROR_STREAM(this->logger_, "###param1");
      node_->set_parameter(rclcpp::Parameter("param2", 4.0));
    }
    if(param.get_name() == "param2"){
      RCLCPP_ERROR_STREAM(this->logger_, "### param2");
    }
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("param_test");
  ParamTest obj(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}