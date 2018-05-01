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

#ifndef COMPOSITION__VISIBILITY_CONTROL_H_
#define COMPOSITION__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define COMPOSITION_EXPORT __attribute__ ((dllexport))
    #define COMPOSITION_IMPORT __attribute__ ((dllimport))
  #else
    #define COMPOSITION_EXPORT __declspec(dllexport)
    #define COMPOSITION_IMPORT __declspec(dllimport)
  #endif
  #ifdef COMPOSITION_BUILDING_DLL
    #define COMPOSITION_PUBLIC COMPOSITION_EXPORT
  #else
    #define COMPOSITION_PUBLIC COMPOSITION_IMPORT
  #endif
  #define COMPOSITION_PUBLIC_TYPE COMPOSITION_PUBLIC
  #define COMPOSITION_LOCAL
#else
  #define COMPOSITION_EXPORT __attribute__ ((visibility("default")))
  #define COMPOSITION_IMPORT
  #if __GNUC__ >= 4
    #define COMPOSITION_PUBLIC __attribute__ ((visibility("default")))
    #define COMPOSITION_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define COMPOSITION_PUBLIC
    #define COMPOSITION_LOCAL
  #endif
  #define COMPOSITION_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // COMPOSITION__VISIBILITY_CONTROL_H_
