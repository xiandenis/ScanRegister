// Portable helpers for robin (GCC __sync_* / OpenMP VLA / M_PI).
#pragma once

#include <atomic>
#include <cstddef>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace robin_compat {

inline long fetch_and_add(long* ptr, long val) {
  return std::atomic_fetch_add(reinterpret_cast<std::atomic<long>*>(ptr), val);
}

inline long fetch_and_add(volatile long* ptr, long val) {
  return std::atomic_fetch_add(reinterpret_cast<std::atomic<long>*>(const_cast<long*>(ptr)), val);
}

inline size_t fetch_and_add(size_t* ptr, size_t val) {
  return std::atomic_fetch_add(reinterpret_cast<std::atomic<size_t>*>(ptr), val);
}

inline size_t fetch_and_sub(size_t* ptr, size_t val) {
  return std::atomic_fetch_sub(reinterpret_cast<std::atomic<size_t>*>(ptr), val);
}

}  // namespace robin_compat

#if defined(__GNUC__) || defined(__clang__)
#define ROBIN_FETCH_AND_ADD(ptr, val) __sync_fetch_and_add((ptr), (val))
#define ROBIN_FETCH_AND_SUB(ptr, val) __sync_fetch_and_sub((ptr), (val))
#else
#define ROBIN_FETCH_AND_ADD(ptr, val) robin_compat::fetch_and_add((ptr), (val))
#define ROBIN_FETCH_AND_SUB(ptr, val) robin_compat::fetch_and_sub((ptr), (val))
#endif
