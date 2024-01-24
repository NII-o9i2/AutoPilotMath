/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 * dingjie <dingjie1@senseauto.com>
 */

#ifndef SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_PLANNING_INCLUDE_PLANNING_UTILS_SPLINE_SMOOTHING_COMMON_MACROS_HPP_
#define SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_PLANNING_INCLUDE_PLANNING_UTILS_SPLINE_SMOOTHING_COMMON_MACROS_HPP_

#include <iostream>
#include <memory>
#include <mutex>
#include <type_traits>
#include <utility>

// // There must be many copy-paste versions of these macros which are same
// // things, undefine them to avoid conflict.
// #undef UNUSED
// #undef DISALLOW_COPY_AND_ASSIGN

// #define UNUSED(param) (void)param

// #define DISALLOW_COPY_AND_ASSIGN(classname) \
//   classname(const classname &) = delete;    \
//   classname &operator=(const classname &) = delete;

// #define DECLARE_SINGLETON(classname)                                      \
//  public:                                                                  \
//   static classname *Instance(bool create_if_needed = true) {              \
//     static classname *instance = nullptr;                                 \
//     if (!instance && create_if_needed) {                                  \
//       static std::once_flag flag;                                         \
//       std::call_once(flag,                                                \
//                      [&] { instance = new (std::nothrow) classname(); }); \
//     }                                                                     \
//     return instance;                                                      \
//   }                                                                       \
//                                                                           \
//   static void CleanUp() {                                                 \
//     auto instance = Instance(false);                                      \
//     if (instance != nullptr) {                                            \
//       CallShutdown(instance);                                             \
//     }                                                                     \
//   }                                                                       \
//                                                                           \
//  private:                                                                 \
//   classname();                                                            \
//   DISALLOW_COPY_AND_ASSIGN(classname)

// #endif  // CYBER_COMMON_MACROS_H_

#undef UNUSED
#undef DISALLOW_COPY_AND_ASSIGN

#define UNUSED(param) (void)param

#define DISALLOW_COPY_AND_ASSIGN(classname) \
  classname(const classname &) = delete;    \
  classname &operator=(const classname &) = delete;

#define DECLARE_SINGLETON(classname)                                      \
 public:                                                                  \
  static classname *Instance(bool create_if_needed = true) {              \
    static classname *instance = nullptr;                                 \
    if (!instance && create_if_needed) {                                  \
      static std::once_flag flag;                                         \
      std::call_once(flag,                                                \
                     [&] { instance = new (std::nothrow) classname(); }); \
    }                                                                     \
    return instance;                                                      \
  }                                                                       \
                                                                          \
 private:                                                                 \
  classname();                                                            \
  DISALLOW_COPY_AND_ASSIGN(classname)

#endif  // SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_PLANNING_INCLUDE_PLANNING_UTILS_SPLINE_SMOOTHING_COMMON_MACROS_HPP_
