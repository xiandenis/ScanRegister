// Forced include (MSVC /FI) or manual include for POSIX/MSVC portability.
#pragma once

#include <cstddef>

#if defined(_MSC_VER)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#ifndef ssize_t
using ssize_t = std::ptrdiff_t;
#endif
#else
#include <unistd.h>
#endif

#ifndef SCAN_REGISTER_ALIGNED
#if defined(_MSC_VER)
#define SCAN_REGISTER_ALIGNED(N) __declspec(align(N))
#else
#define SCAN_REGISTER_ALIGNED(N) __attribute__((aligned(N)))
#endif
#endif
