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

#ifdef __clang__
// TODO(dirk-thomas) custom implementation until we can use libc++ 3.9
#include <string>
namespace fs
{
class path
{
public:
  explicit path(const std::string & p)
  : path_(p)
  {}
  bool is_absolute()
  {
    return path_[0] == '/';
  }

private:
  std::string path_;
};
}  // namespace fs
#else
# include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#endif

#include <cstring>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "ament_index_cpp/get_resource.hpp"
#include "class_loader/class_loader.h"
#include "composition/srv/load_node.hpp"
#include "rclcpp/rclcpp.hpp"


std::vector<std::string> split(
  const std::string & s, char delim, bool skip_empty = false)
{
  std::vector<std::string> result;
  std::stringstream ss;
  ss.str(s);
  std::string item;
  while (std::getline(ss, item, delim)) {
    if (skip_empty && item == "") {
      continue;
    }
    result.push_back(item);
  }
  return result;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("api_composition");

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);

  std::vector<class_loader::ClassLoader *> loaders;
  std::vector<std::shared_ptr<rclcpp::Node>> nodes;

  auto server = node->create_service<composition::srv::LoadNode>(
    "load_node",
    [&exec, &loaders, &nodes](
      const std::shared_ptr<rmw_request_id_t>,
      const std::shared_ptr<composition::srv::LoadNode::Request> request,
      std::shared_ptr<composition::srv::LoadNode::Response> response)
  {
    // get node plugin resource from package
    std::string content;
    std::string base_path;
    if (!ament_index_cpp::get_resource("node_plugin", request->package_name, content, &base_path)) {
      fprintf(stderr, "Could not find requested resource in ament index\n");
      response->success = false;
      return;
    }

    std::vector<std::string> lines = split(content, '\n', true);
    for (auto line : lines) {
      std::vector<std::string> parts = split(line, ';');
      if (parts.size() != 2) {
        fprintf(stderr, "Invalid resource entry\n");
        response->success = false;
        return;
      }
      // match plugin name with the same rmw suffix as this executable
      if (parts[0] != request->plugin_name) {
        continue;
      }

      std::string class_name = parts[0];

      // load node plugin
      std::string library_path = parts[1];
      if (!fs::path(library_path).is_absolute()) {
        library_path = base_path + "/" + library_path;
      }
      printf("Load library %s\n", library_path.c_str());
      class_loader::ClassLoader * loader;
      try {
        loader = new class_loader::ClassLoader(library_path);
      } catch (const std::exception & ex) {
        fprintf(stderr, "Failed to load library: %s\n", ex.what());
        response->success = false;
        return;
      } catch (...) {
        fprintf(stderr, "Failed to load library\n");
        response->success = false;
        return;
      }
      auto classes = loader->getAvailableClasses<rclcpp::Node>();
      for (auto clazz : classes) {
        if (clazz == class_name) {
          printf("Instantiate class %s\n", clazz.c_str());
          auto node = loader->createInstance<rclcpp::Node>(clazz);
          exec.add_node(node);
          nodes.push_back(node);
          loaders.push_back(loader);
          response->success = true;
          return;
        }
      }

      // no matching class found in loader
      delete loader;
      fprintf(
        stderr, "Failed to find class with the requested plugin name '%s' in "
        "the loaded library\n",
        request->plugin_name.c_str());
      response->success = false;
      return;
    }
    fprintf(
      stderr, "Failed to find plugin name '%s' in prefix '%s'\n",
      request->plugin_name.c_str(), base_path.c_str());
    response->success = false;
  });

  exec.spin();

  for (auto node : nodes) {
    exec.remove_node(node);
  }
  nodes.clear();

  rclcpp::shutdown();

  return 0;
}
