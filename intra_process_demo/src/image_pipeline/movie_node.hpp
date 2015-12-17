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

#ifndef IMAGE_PIPELINE__MOVIE_NODE_HPP_
#define IMAGE_PIPELINE__MOVIE_NODE_HPP_

#include <chrono>
#include <sstream>
#include <string>
#include <thread>

#include "opencv2/highgui/highgui.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "common.hpp"
#include "video_node.hpp"

// Node which captures images from a movie file using OpenCV and publishes them.
// Images are annotated with this process's id as well as the message's ptr.
class MovieNode : public VideoNode
{
public:
  MovieNode(const std::string & output, const std::string & file_name,
    const std::string & node_name = "camera_node")
  : VideoNode(output, node_name)
  {
    // Initialize OpenCV
    cap_.open(file_name);
    if (!cap_.isOpened()) {
      throw std::runtime_error("Could not open video file!");
    }
    // Create a publisher on the output topic.
    pub_ = this->create_publisher<sensor_msgs::msg::Image>(output, rmw_qos_profile_sensor_data);
    // Create the camera reading loop.
    thread_ = std::thread(std::bind(&VideoNode::loop, this));
  }
};

#endif  // IMAGE_PIPELINE__MOVIE_NODE_HPP_
