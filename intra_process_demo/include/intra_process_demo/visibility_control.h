#ifndef INTRA_PROCESS_DEMO__VISIBILITY_CONTROL_H_
#define INTRA_PROCESS_DEMO__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define INTRA_PROCESS_DEMO_EXPORT __attribute__ ((dllexport))
    #define INTRA_PROCESS_DEMO_IMPORT __attribute__ ((dllimport))
  #else
    #define INTRA_PROCESS_DEMO_EXPORT __declspec(dllexport)
    #define INTRA_PROCESS_DEMO_IMPORT __declspec(dllimport)
  #endif
  #ifdef INTRA_PROCESS_DEMO_BUILDING_LIBRARY
    #define INTRA_PROCESS_DEMO_PUBLIC INTRA_PROCESS_DEMO_EXPORT
  #else
    #define INTRA_PROCESS_DEMO_PUBLIC INTRA_PROCESS_DEMO_IMPORT
  #endif
  #define INTRA_PROCESS_DEMO_PUBLIC_TYPE INTRA_PROCESS_DEMO_PUBLIC
  #define INTRA_PROCESS_DEMO_LOCAL
#else
  #define INTRA_PROCESS_DEMO_EXPORT __attribute__ ((visibility("default")))
  #define INTRA_PROCESS_DEMO_IMPORT
  #if __GNUC__ >= 4
    #define INTRA_PROCESS_DEMO_PUBLIC __attribute__ ((visibility("default")))
    #define INTRA_PROCESS_DEMO_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define INTRA_PROCESS_DEMO_PUBLIC
    #define INTRA_PROCESS_DEMO_LOCAL
  #endif
  #define INTRA_PROCESS_DEMO_PUBLIC_TYPE
#endif

#endif  // INTRA_PROCESS_DEMO__VISIBILITY_CONTROL_H_
