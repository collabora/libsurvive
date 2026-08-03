#pragma once
#include <stddef.h>
#include <math.h>
static inline double __cnkalman_safe_sqrt(double x) { return x > 0 ? sqrt(x) : 0; }
#define sqrt __cnkalman_safe_sqrt
static inline double __cnkalman_safe_asin(double x) { return asin( (x > 1) ? 1 : ((x < -1) ? -1 : x)); }
#define asin __cnkalman_safe_asin
#ifndef WIN32
#ifndef ANDROID
#include <complex.h>
static inline double __cnkalman_safe_pow(double x, double y) { return x >= 0 ? pow(x, y) : creal(cpow(x, y)); }
#define pow __cnkalman_safe_pow
#endif
#endif
#define GEN_FLT FLT
