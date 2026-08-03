#ifdef __cplusplus
extern "C" {
#endif

#include "assert.h"
#include "stdint.h"
#include "stdlib.h"

#define CN_Error(code, msg) assert(0 && msg); // cv::error( code, msg, CN_Func, __FILE__, __LINE__ )

#define CN_32FC1 CN_32F
#define CN_64FC1 CN_64F

#define CN_MAGIC_MASK 0xFFFF0000
#define CN_MAT_MAGIC_VAL 0x42420000

#define CN_CN_MAX 512
#define CN_CN_SHIFT 3
#define CN_DEPTH_MAX (1 << CN_CN_SHIFT)

#define CN_MAT_DEPTH_MASK (CN_DEPTH_MAX - 1)
#define CN_MAT_DEPTH(flags) ((flags)&CN_MAT_DEPTH_MASK)

#define CN_MAKETYPE(depth, cn) (CN_MAT_DEPTH(depth) + (((cn)-1) << CN_CN_SHIFT))
#define CN_MAKE_TYPE CN_MAKETYPE

#define CN_MAT_CN_MASK ((CN_CN_MAX - 1) << CN_CN_SHIFT)
#define CN_MAT_CN(flags) ((((flags)&CN_MAT_CN_MASK) >> CN_CN_SHIFT) + 1)
#define CN_MAT_TYPE_MASK (CN_DEPTH_MAX * CN_CN_MAX - 1)
#define CN_MAT_TYPE(flags) ((flags)&CN_MAT_TYPE_MASK)
#define CN_MAT_CONT_FLAG_SHIFT 14
#define CN_MAT_CONT_FLAG (1 << CN_MAT_CONT_FLAG_SHIFT)
#define CN_IS_MAT_CONT(flags) ((flags)&CN_MAT_CONT_FLAG)
#define CN_IS_CONT_MAT CN_IS_MAT_CONT
#define CN_SUBMAT_FLAG_SHIFT 15
#define CN_SUBMAT_FLAG (1 << CN_SUBMAT_FLAG_SHIFT)
#define CN_IS_SUBMAT(flags) ((flags)&CN_MAT_SUBMAT_FLAG)

#define CN_IS_MATND_HDR(mat) ((mat) != NULL && (((const CnMat *)(mat))->type & CN_MAGIC_MASK) == CN_MATND_MAGIC_VAL)

#define CN_IS_MATND(mat) (CN_IS_MATND_HDR(mat) && ((const CnMat *)(mat))->data.ptr != NULL)
#define CN_MATND_MAGIC_VAL 0x42430000

/** 0x3a50 = 11 10 10 01 01 00 00 ~ array of log2(sizeof(arr_type_elem)) */
#define CN_ELEM_SIZE(type)                                                                                             \
	(CN_MAT_CN(type) << ((((sizeof(size_t) / 4 + 1) * 16384 | 0x3a50) >> CN_MAT_DEPTH(type) * 2) & 3))

#ifndef MIN
#define MIN(a, b) ((a) > (b) ? (b) : (a))
#endif

#ifndef MAX
#define MAX(a, b) ((a) < (b) ? (b) : (a))
#endif

/** 0x3a50 = 11 10 10 01 01 00 00 ~ array of log2(sizeof(arr_type_elem)) */
#define CN_ELEM_SIZE(type)                                                                                             \
	(CN_MAT_CN(type) << ((((sizeof(size_t) / 4 + 1) * 16384 | 0x3a50) >> CN_MAT_DEPTH(type) * 2) & 3))

//#include "shim_types_c.h"

#ifdef __cplusplus
}
#endif