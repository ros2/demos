// Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#ifndef QUALITY_OF_SERVICE_DEMO__UTILS_H_
#define QUALITY_OF_SERVICE_DEMO__UTILS_H_

#include <atomic>
#include <thread>

#include <rmw/types.h>

double
rmw_time_to_seconds(const rmw_time_t & time);

void
print_qos(const rmw_qos_profile_t & qos);

class
  CommandGetter
{
public:
  bool is_active() const;

  void start();
  void stop();

  void operator()() const;
  virtual void handle_cmd(const char cmd) const = 0;

private:
  char getch() const;

  std::thread thread_;
  std::atomic<bool> run_;
};

#endif  // QUALITY_OF_SERVICE_DEMO__UTILS_H_
