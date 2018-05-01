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

#ifndef LOGGING_DEMO__VISIBILITY_CONTROL_H_
#define LOGGING_DEMO__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define LOGGING_DEMO_EXPORT __attribute__ ((dllexport))
    #define LOGGING_DEMO_IMPORT __attribute__ ((dllimport))
  #else
    #define LOGGING_DEMO_EXPORT __declspec(dllexport)
    #define LOGGING_DEMO_IMPORT __declspec(dllimport)
  #endif
  #ifdef LOGGING_DEMO_BUILDING_DLL
    #define LOGGING_DEMO_PUBLIC LOGGING_DEMO_EXPORT
  #else
    #define LOGGING_DEMO_PUBLIC LOGGING_DEMO_IMPORT
  #endif
  #define LOGGING_DEMO_PUBLIC_TYPE LOGGING_DEMO_PUBLIC
  #define LOGGING_DEMO_LOCAL
#else
  #define LOGGING_DEMO_EXPORT __attribute__ ((visibility("default")))
  #define LOGGING_DEMO_IMPORT
  #if __GNUC__ >= 4
    #define LOGGING_DEMO_PUBLIC __attribute__ ((visibility("default")))
    #define LOGGING_DEMO_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define LOGGING_DEMO_PUBLIC
    #define LOGGING_DEMO_LOCAL
  #endif
  #define LOGGING_DEMO_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // LOGGING_DEMO__VISIBILITY_CONTROL_H_
