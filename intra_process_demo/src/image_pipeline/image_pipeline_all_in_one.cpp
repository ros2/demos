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

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "camera_node.hpp"
#include "image_view_node.hpp"
#include "movie_node.hpp"
#include "watermark_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;

  std::vector<std::string> args(argv + 1, argv + argc);

  bool show_image = true;
  std::string video_file = "";

  auto it = std::find(args.begin(), args.end(), "-f");
  if (it != args.end()) {
    video_file = *(it + 1);
  }

  it = std::find(args.begin(), args.end(), "-s");
  if (it != args.end()) {
    // 0 = do not show video feed, 1 = show video feed
    show_image = std::stoul(*(it + 1)) == 1;
  }

  // Connect the nodes as a pipeline: camera_node -> watermark_node -> image_view_node
  if (video_file.empty()) {
    auto camera_node = std::make_shared<CameraNode>("image");
    executor.add_node(camera_node);
  } else {
    auto movie_node = std::make_shared<MovieNode>("image", video_file);
    executor.add_node(movie_node);
  }

  auto watermark_node =
    std::make_shared<WatermarkNode>("image", "watermarked_image", "Hello world!");
  auto image_view_node = std::make_shared<ImageViewNode>("watermarked_image", show_image);

  executor.add_node(watermark_node);
  executor.add_node(image_view_node);

  executor.spin();
  return 0;
}
