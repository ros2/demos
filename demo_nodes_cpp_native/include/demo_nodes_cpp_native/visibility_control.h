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

#ifndef DEMO_NODES_CPP_NATIVE__VISIBILITY_CONTROL_H_
#define DEMO_NODES_CPP_NATIVE__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define DEMO_NODES_CPP_NATIVE_EXPORT __attribute__ ((dllexport))
    #define DEMO_NODES_CPP_NATIVE_IMPORT __attribute__ ((dllimport))
  #else
    #define DEMO_NODES_CPP_NATIVE_EXPORT __declspec(dllexport)
    #define DEMO_NODES_CPP_NATIVE_IMPORT __declspec(dllimport)
  #endif
  #ifdef DEMO_NODES_CPP_NATIVE_BUILDING_DLL
    #define DEMO_NODES_CPP_NATIVE_PUBLIC DEMO_NODES_CPP_NATIVE_EXPORT
  #else
    #define DEMO_NODES_CPP_NATIVE_PUBLIC DEMO_NODES_CPP_NATIVE_IMPORT
  #endif
  #define DEMO_NODES_CPP_NATIVE_PUBLIC_TYPE DEMO_NODES_CPP_NATIVE_PUBLIC
  #define DEMO_NODES_CPP_NATIVE_LOCAL
#else
  #define DEMO_NODES_CPP_NATIVE_EXPORT __attribute__ ((visibility("default")))
  #define DEMO_NODES_CPP_NATIVE_IMPORT
  #if __GNUC__ >= 4
    #define DEMO_NODES_CPP_NATIVE_PUBLIC __attribute__ ((visibility("default")))
    #define DEMO_NODES_CPP_NATIVE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define DEMO_NODES_CPP_NATIVE_PUBLIC
    #define DEMO_NODES_CPP_NATIVE_LOCAL
  #endif
  #define DEMO_NODES_CPP_NATIVE_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // DEMO_NODES_CPP_NATIVE__VISIBILITY_CONTROL_H_
