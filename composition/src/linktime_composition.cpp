// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include <memory>
#include <string>
#include <vector>

#include "class_loader/class_loader.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  std::vector<class_loader::ClassLoader *> loaders;
  std::vector<std::shared_ptr<rclcpp::Node>> nodes;

  std::vector<std::string> libraries = {
    // all classes from libraries linked by the linker (rather then dlopen)
    // are registered under the library_path ""
    "",
  };
  for (auto library : libraries) {
    printf("Load library %s\n", library.c_str());
    auto loader = new class_loader::ClassLoader(library);
    auto classes = loader->getAvailableClasses<rclcpp::Node>();
    for (auto clazz : classes) {
      printf("Instantiate class %s\n", clazz.c_str());
      auto node = loader->createInstance<rclcpp::Node>(clazz);
      exec.add_node(node);
      nodes.push_back(node);
    }
    loaders.push_back(loader);
  }

  exec.spin();

  for (auto node : nodes) {
    exec.remove_node(node);
  }
  nodes.clear();

  rclcpp::shutdown();

  return 0;
}
