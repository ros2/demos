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

#ifndef QUALITY_OF_SERVICE_DEMO__VISIBILITY_CONTROL_H_
#define QUALITY_OF_SERVICE_DEMO__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define QUALITY_OF_SERVICE_DEMO_EXPORT __attribute__ ((dllexport))
    #define QUALITY_OF_SERVICE_DEMO_IMPORT __attribute__ ((dllimport))
  #else
    #define QUALITY_OF_SERVICE_DEMO_EXPORT __declspec(dllexport)
    #define QUALITY_OF_SERVICE_DEMO_IMPORT __declspec(dllimport)
  #endif
  #ifdef QUALITY_OF_SERVICE_DEMO_BUILDING_DLL
    #define QUALITY_OF_SERVICE_DEMO_PUBLIC QUALITY_OF_SERVICE_DEMO_EXPORT
  #else
    #define QUALITY_OF_SERVICE_DEMO_PUBLIC QUALITY_OF_SERVICE_DEMO_IMPORT
  #endif
  #define QUALITY_OF_SERVICE_DEMO_PUBLIC_TYPE QUALITY_OF_SERVICE_DEMO_PUBLIC
  #define QUALITY_OF_SERVICE_DEMO_LOCAL
#else
  #define QUALITY_OF_SERVICE_DEMO_EXPORT __attribute__ ((visibility("default")))
  #define QUALITY_OF_SERVICE_DEMO_IMPORT
  #if __GNUC__ >= 4
    #define QUALITY_OF_SERVICE_DEMO_PUBLIC __attribute__ ((visibility("default")))
    #define QUALITY_OF_SERVICE_DEMO_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define QUALITY_OF_SERVICE_DEMO_PUBLIC
    #define QUALITY_OF_SERVICE_DEMO_LOCAL
  #endif
  #define QUALITY_OF_SERVICE_DEMO_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // QUALITY_OF_SERVICE_DEMO__VISIBILITY_CONTROL_H_
