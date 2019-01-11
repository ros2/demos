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

#ifdef _WIN32
#include <Windows.h>  // For GetTickCount().
#endif

#include <cstring>
#include <cstdio>
#include <string>
#include <vector>

#include "./burger.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using burger::Burger;  // i've always wanted to write that

#ifndef _WIN32
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wchar-subscripts"
#endif
// here lies the world's slowest portable base64 decoder
void decode_base64(const char * cstr, std::vector<uint8_t> & out)
{
  int len = static_cast<int>(strlen(cstr));
  if (len < 2) {
    return;  // would have to think too hard about trivial inputs
  }
  out.resize(len * 3 / 4);  // deal with padding bytes later
  uint8_t base64_map[256] = {0};
  for (uint8_t i = 'A'; i <= 'Z'; i++) {
    base64_map[i] = i - 'A';
  }
  for (uint8_t i = 'a'; i <= 'z'; i++) {
    base64_map[i] = i - 'a' + 26;
  }
  for (uint8_t i = '0'; i <= '9'; i++) {
    base64_map[i] = i - '0' + 52;
  }
  base64_map['+'] = 62;
  base64_map['/'] = 63;
  int ridx = 0, widx = 0;
  for (ridx = 0; ridx < len; ridx += 4) {
    // we expand each group of 4 code bytes into 3 output bytes
    uint32_t block = 0;
    block = (base64_map[cstr[ridx]] << 18) |
      (base64_map[cstr[ridx + 1]] << 12) |
      (base64_map[cstr[ridx + 2]] << 6) |
      (base64_map[cstr[ridx + 3]] << 0);
    out[widx++] = (block >> 16) & 0xff;
    out[widx++] = (block >> 8) & 0xff;
    out[widx++] = (block >> 0) & 0xff;
  }
  // fix padding now. (these branches are untested so they're probably wrong)
  if (cstr[len - 1] == '=' && cstr[len - 2] == '=') {
    // there were two padding bytes. remove the two last output bytes
    out.pop_back();
    out.pop_back();
  } else if (cstr[len - 1] == '=') {
    // there was only one padding byte. remove the last output byte.
    out.pop_back();
  }
}
#ifndef _WIN32
#pragma GCC diagnostic pop
#endif

Burger::Burger()
{
  size_t burger_size = strlen(BURGER);
  std::vector<uint8_t> burger_png;
  burger_png.resize(burger_size);
  decode_base64(BURGER, burger_png);
  burger_template = cv::imdecode(burger_png, cv::ImreadModes::IMREAD_COLOR);
  cv::floodFill(burger_template, cv::Point(1, 1), CV_RGB(1, 1, 1));
  cv::compare(burger_template, 1, burger_mask, cv::CMP_NE);
#ifndef _WIN32
  srand(time(NULL));
#else
  srand(GetTickCount());
#endif
}

cv::Mat & Burger::render_burger(size_t width, size_t height)
{
  int width_i = static_cast<int>(width);
  int height_i = static_cast<int>(height);
  if (width_i < burger_template.size().width || height_i < burger_template.size().height) {
    std::string msg = "Target resolution must be at least the burger size (" +
      std::to_string(burger_template.size().width) + " x " +
      std::to_string(burger_template.size().height) + ")";
    throw std::runtime_error(msg.c_str());
  }
  if (burger_buf.size().width != width_i || burger_buf.size().height != height_i) {
    int num_burgers = rand() % 10 + 2;  // NOLINT
    x.resize(num_burgers);
    y.resize(num_burgers);
    x_inc.resize(num_burgers);
    y_inc.resize(num_burgers);
    for (int b = 0; b < num_burgers; b++) {
      if (width - burger_template.size().width > 0) {
        x[b] = rand() % (width - burger_template.size().width);  // NOLINT
      } else {
        x[b] = 0;
      }
      if (height - burger_template.size().height > 0) {
        y[b] = rand() % (height - burger_template.size().height);  // NOLINT
      } else {
        y[b] = 0;
      }
      x_inc[b] = rand() % 3 + 1;  // NOLINT
      y_inc[b] = rand() % 3 + 1;  // NOLINT
    }
    burger_buf = cv::Mat(height_i, width_i, CV_8UC3);
  }
  burger_buf = cv::Scalar(0, 0, 0);
  for (int b = 0; b < static_cast<int>(x.size()); b++) {
    burger_template.copyTo(burger_buf(cv::Rect(
        x[b],
        y[b],
        burger_template.size().height,
        burger_template.size().width
      )), burger_mask);
    x[b] += x_inc[b];
    y[b] += y_inc[b];
    // bounce as needed
    if (x[b] < 0 || x[b] > width_i - burger_template.size().width - 1) {
      x_inc[b] *= -1;
      if (x[b] < 0) {
        x[b] = 0;
      } else {
        x[b] = width_i - burger_template.size().width;
      }
    }
    if (y[b] < 0 || y[b] > height_i - burger_template.size().height - 1) {
      y_inc[b] *= -1;
      if (y[b] < 0) {
        y[b] = 0;
      } else {
        y[b] = height_i - burger_template.size().height;
      }
    }
  }
  return burger_buf;
}
