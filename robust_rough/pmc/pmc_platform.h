/**
 * Cross-platform helpers for PMC (Windows / Linux).
 */
#ifndef PMC_PLATFORM_H_
#define PMC_PLATFORM_H_

#include <cstddef>

#if defined(_WIN32) || defined(_WIN64)
#define PMC_PLATFORM_WINDOWS 1
#else
#define PMC_PLATFORM_WINDOWS 0
#endif

/* OpenMP: use stubs when not compiled with -fopenmp / /openmp */
#if defined(_OPENMP)
#include <omp.h>
#else
inline int pmc_omp_get_max_threads() { return 1; }
inline int pmc_omp_get_thread_num() { return 0; }
inline void pmc_omp_set_num_threads(int) {}
#define omp_get_max_threads pmc_omp_get_max_threads
#define omp_get_thread_num pmc_omp_get_thread_num
#define omp_set_num_threads pmc_omp_set_num_threads
#endif

/* POSIX getopt: provided by unistd on Linux; implemented on Windows */
#if PMC_PLATFORM_WINDOWS
extern int opterr;
extern int optind;
extern char* optarg;
int pmc_getopt(int argc, char* const argv[], const char* optstring);
#define getopt pmc_getopt
#else
#include <unistd.h>
#endif

double pmc_get_time_seconds();

#endif /* PMC_PLATFORM_H_ */
