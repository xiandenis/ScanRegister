/**
 * Cross-platform helpers for PMC (Windows / Linux).
 */
#include "pmc_platform.h"

#include <chrono>
#include <cerrno>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#if PMC_PLATFORM_WINDOWS
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <dirent.h>
#include <sys/time.h>
#endif

double pmc_get_time_seconds() {
#if PMC_PLATFORM_WINDOWS
  using clock = std::chrono::steady_clock;
  static const auto t0 = clock::now();
  const auto elapsed = clock::now() - t0;
  return std::chrono::duration<double>(elapsed).count();
#else
  timeval t{};
  gettimeofday(&t, nullptr);
  return static_cast<double>(t.tv_sec) + static_cast<double>(t.tv_usec) / 1e6;
#endif
}

#if PMC_PLATFORM_WINDOWS

int opterr = 1;
int optind = 1;
char* optarg = nullptr;

int pmc_getopt(int argc, char* const argv[], const char* optstring) {
  if (optind >= argc || argv[optind] == nullptr || argv[optind][0] != '-') {
    return -1;
  }
  if (argv[optind][1] == '-' && argv[optind][2] == '\0') {
    ++optind;
    return -1;
  }

  const char opt = argv[optind][1];
  if (opt == ':' || opt == '\0') {
    return '?';
  }

  const char* place = std::strchr(optstring, opt);
  if (place == nullptr) {
    return '?';
  }

  if (place[1] == ':') {
    if (argv[optind][2] != '\0') {
      optarg = &argv[optind][2];
    } else if (++optind < argc) {
      optarg = argv[optind];
    } else {
      return '?';
    }
  }

  ++optind;
  return opt;
}

#endif /* PMC_PLATFORM_WINDOWS */

/* memory_usage / getdir live in pmc_utils.cpp but use platform branches there */
