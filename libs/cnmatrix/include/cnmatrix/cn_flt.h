#pragma once

#ifdef CN_USE_FLOAT

#define FLT float
#define FLT_SQRT sqrtf
#define FLT_TAN tanf
#define FLT_SIN sinf
#define FLT_COS cosf
#define FLT_ACOS acosf
#define FLT_ASIN asinf
#define FLT_ATAN2 atan2f
#define FLT_FABS__ fabsf
#define FLT_STRTO strtof

#define CN_FLT CN_32F
#define FLT_POW powf

#else

#define CN_USE_DOUBLE 1
#define FLT double
#define FLT_SQRT sqrt
#define FLT_TAN tan
#define FLT_SIN sin
#define FLT_COS cos
#define FLT_ACOS acos
#define FLT_ASIN asin
#define FLT_ATAN2 atan2
#define FLT_FABS__ fabs
#define FLT_STRTO strtod

#endif
