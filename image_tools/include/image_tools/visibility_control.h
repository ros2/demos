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

#ifndef IMAGE_TOOLS__VISIBILITY_CONTROL_H_
#define IMAGE_TOOLS__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define IMAGE_TOOLS_EXPORT __attribute__ ((dllexport))
    #define IMAGE_TOOLS_IMPORT __attribute__ ((dllimport))
  #else
    #define IMAGE_TOOLS_EXPORT __declspec(dllexport)
    #define IMAGE_TOOLS_IMPORT __declspec(dllimport)
  #endif
  #ifdef IMAGE_TOOLS_BUILDING_DLL
    #define IMAGE_TOOLS_PUBLIC IMAGE_TOOLS_EXPORT
  #else
    #define IMAGE_TOOLS_PUBLIC IMAGE_TOOLS_IMPORT
  #endif
  #define IMAGE_TOOLS_PUBLIC_TYPE IMAGE_TOOLS_PUBLIC
  #define IMAGE_TOOLS_LOCAL
#else
  #define IMAGE_TOOLS_EXPORT __attribute__ ((visibility("default")))
  #define IMAGE_TOOLS_IMPORT
  #if __GNUC__ >= 4
    #define IMAGE_TOOLS_PUBLIC __attribute__ ((visibility("default")))
    #define IMAGE_TOOLS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define IMAGE_TOOLS_PUBLIC
    #define IMAGE_TOOLS_LOCAL
  #endif
  #define IMAGE_TOOLS_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // IMAGE_TOOLS__VISIBILITY_CONTROL_H_
