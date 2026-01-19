#pragma once

#ifndef CN_EXPORT_FUNCTION
#ifdef _WIN32
#define CN_EXPORT_FUNCTION __declspec(dllexport)
#define CN_EXPORT_CLASS CN_EXPORT_FUNCTION
#define CN_IMPORT_FUNCTION __declspec(dllimport)
#else
#define CN_EXPORT_FUNCTION __attribute__((visibility("default")))
#define CN_EXPORT_CLASS CN_EXPORT_FUNCTION
#define CN_IMPORT_FUNCTION CN_EXPORT_FUNCTION
#endif
#endif

/**
 * Work around an issue in MSVC where NAN is not a constant expression
 */
#ifdef _MSC_VER
#define _UCRT_NOISY_NAN
#endif

#include <stdbool.h>

#ifdef __APPLE__
#include <stdlib.h>
#else
#include <malloc.h>
#endif
#include "cnmatrix/cn_flt.h"
#include "math.h"
#include "string.h"
#include <assert.h>

#define CN_FLT_PTR(m) ((FLT *)((m)->data))
#define CN_RAW_PTR(m) ((FLT *)((m)->data))

#ifndef M_PI
# define M_PI		3.14159265358979323846
#endif

#ifdef __cplusplus
extern "C" {
static inline FLT cnMatrixGet(const struct CnMat *mat, int row, int col);
static inline size_t cn_matrix_idx(const struct CnMat *mat, int row, int col);
#define CN_MATRIX_DEFAULT(x) = (x)
}
#endif
#ifndef CN_MATRIX_DEFAULT
#define CN_MATRIX_DEFAULT(x)
#endif

typedef struct CnMat {
	int step;

	FLT *data;

	int rows;
	int cols;
#ifdef __cplusplus
    FLT operator()(int i, int j) const { return cnMatrixGet(this, i, j);}
    FLT& operator()(int i, int j) { return data[cn_matrix_idx(this, i, j)]; }
    FLT operator()(int i) const { assert(cols == 1); return cnMatrixGet(this, i, 0);}
    FLT& operator()(int i) { assert(cols == 1); return data[i]; }
#endif
} CnMat;

#ifdef USE_EIGEN
#include "cn_matrix.eigen.h"
#else
#include "cn_matrix.blas.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
// Avoids missing-field-initializers errors
#define ZERO_INITIALIZATION                                                                                            \
	{}
#else
#define ZERO_INITIALIZATION                                                                                            \
	{ 0 }
#endif

CnMat *cnInitMatHeader(CnMat *arr, int rows, int cols);
CnMat *cnCreateMat(int height, int width);
  
enum cnInvertMethod {
	CN_INVERT_METHOD_UNKNOWN = 0,
	CN_INVERT_METHOD_SVD = 1,
	CN_INVERT_METHOD_LU = 2,
	CN_INVERT_METHOD_QR = 3,
};

double cnInvert(const CnMat *srcarr, CnMat *dstarr, enum cnInvertMethod method);
void cnSqRootSymmetric(const CnMat *srcarr, CnMat *dstarr);
void cnRand(CnMat *dstarr, FLT mu, FLT sigma);

enum cnGEMMFlags {
	CN_GEMM_FLAG_A_T = 1,
	CN_GEMM_FLAG_B_T = 2,
	CN_GEMM_FLAG_C_T = 4,
};

void cnGEMM(const CnMat *src1, const CnMat *src2, double alpha, const CnMat *src3, double beta, CnMat *dst,
			enum cnGEMMFlags tABC);

FLT cnNorm(const CnMat *s);
FLT cnNorm2(const CnMat *s);
FLT cnDistance(const CnMat *a, const CnMat *b);
void cnSub(CnMat *dest, const CnMat *a, const CnMat *b);
void cnAdd(CnMat *dest, const CnMat *a, const CnMat *b);
void cnAddScaled(CnMat *dest, const CnMat *a, FLT as, const CnMat *b, FLT bs);
void cnScale(CnMat *dest, const CnMat *a, FLT s);
void cnElementwiseMultiply(CnMat *dest, const CnMat *a, const CnMat *b);
  FLT cnDot(const CnMat* a, const CnMat* b);
  
/**
 * xarr = argmin_x(Aarr * x - Barr)
 */
int cnSolve(const CnMat *Aarr, const CnMat *Barr, CnMat *xarr, enum cnInvertMethod method);

void cnSetZero(CnMat *arr);

void cnCopy(const CnMat *src, CnMat *dest, const CnMat *mask CN_MATRIX_DEFAULT(0));
void cnCopyROI(const CnMat *src, CnMat *dest, int start_i, int start_j);

CnMat *cnCloneMat(const CnMat *mat);

void cnReleaseMat(CnMat **mat);

enum cnSVDFlags { CN_SVD_MODIFY_A = 1, CN_SVD_U_T = 2, CN_SVD_V_T = 4 };

void cnSVD(CnMat *aarr, CnMat *warr, CnMat *uarr, CnMat *varr, enum cnSVDFlags flags);

void cnMulTransposed(const CnMat *src, CnMat *dst, int order, const CnMat *delta, double scale);

void cnTranspose(const CnMat *M, CnMat *dst);

double cnDet(const CnMat *M);

#ifdef CN_MATRIX_USE_MALLOC
#define CN_MATRIX_ALLOC(size) calloc(1, size)
#define CN_MATRIX_FREE(ptr) free(ptr)
#define CN_MATRIX_STACK_SCOPE_BEGIN {
#define CN_MATRIX_STACK_SCOPE_END }
#else
#define CN_MATRIX_ALLOC(size) (memset(alloca(size), 0, size))
#define CN_MATRIX_FREE(ptr)
#define CN_MATRIX_STACK_SCOPE_BEGIN
#define CN_MATRIX_STACK_SCOPE_END
#endif

#define CN_CREATE_STACK_VEC(name, rows)					\
  CN_MATRIX_STACK_SCOPE_BEGIN						\
  FLT *_##name = (FLT*)CN_MATRIX_ALLOC((rows) * sizeof(FLT));	\
  CnMat name = cnVec(rows, _##name);

#define CN_CREATE_STACK_MAT(name, rows, cols)                                                                          \
	CN_MATRIX_STACK_SCOPE_BEGIN                                                                                        \
	FLT *_##name = (FLT*)CN_MATRIX_ALLOC((rows) * (cols) * sizeof(FLT)); \
	CnMat name = cnMat(rows, cols, _##name);

#define CN_FREE_STACK_MAT(name)                                                                                        \
	CN_MATRIX_FREE(_##name);                                                                                           \
	CN_MATRIX_STACK_SCOPE_END

#ifndef CN_MATRIX_IS_COL_MAJOR
static inline int cn_stride(const struct CnMat *m) { return m->rows; }
#else
static inline int cn_stride(const struct CnMat *m) { return m->cols; }
#endif

static inline void cn_set_zero(struct CnMat *m) { memset(CN_FLT_PTR(m), 0, sizeof(FLT) * m->rows * m->cols); }
static inline void cn_set_constant(struct CnMat *m, FLT v) {
	for (int i = 0; i < m->rows * m->cols; i++)
		CN_FLT_PTR(m)[i] = v;
}

static inline bool cn_is_finite(const struct CnMat *m) {
	for (int i = 0; i < m->rows * m->cols; i++)
		if (!isfinite(CN_FLT_PTR(m)[i]))
			return false;
	return true;
}

static inline void cn_matrix_copy(struct CnMat *dst, const struct CnMat *src) {
	cnCopy(src, dst, 0);
}

static inline FLT *cn_as_vector(struct CnMat *m) {
	assert(m->rows == 1 || m->cols == 1);
	return CN_FLT_PTR(m);
}

static inline const FLT *cn_as_const_vector(const struct CnMat *m) {
	assert(m->rows == 1 || m->cols == 1);
	return CN_FLT_PTR(m);
}

/** Inline constructor. No data is allocated internally!!!
 * (Use together with cnCreateData, or use cnCreateMat instead to
 * get a matrix with allocated data):
 */
static inline CnMat cnMat(int rows, int cols, FLT *data) {
	CnMat m = ZERO_INITIALIZATION;

	m.cols = cols;
	m.rows = rows;
#ifndef CN_MATRIX_IS_COL_MAJOR
	m.step = m.cols;
#else
	m.step = m.rows;
#endif

	if (!data) {
		m.data = (FLT *)calloc(m.cols * m.rows, sizeof(FLT));
	} else {
		m.data = (FLT *)data;
	}

#if SURVIVE_ASAN_CHECKS
	volatile double v = cvmGet(&m, rows - 1, cols - 1);
	(void)v;
#endif

	return m;
}

  static inline CnMat cnMatCalloc(int height, int width) {
    return cnMat(height, width, (FLT *)calloc(height * width, sizeof(FLT)));
  }

static inline CnMat cnVec(int rows, FLT *data) { return cnMat(rows, 1, data); }

static inline size_t cn_matrix_idx(const CnMat *mat, int row, int col) {
	assert((unsigned)row < (unsigned)mat->rows && (unsigned)col < (unsigned)mat->cols);
#ifndef CN_MATRIX_IS_COL_MAJOR
	return (size_t)mat->step * row + col;
#else
	return (size_t)mat->step * col + row;
#endif
}

static inline CnMat cnMatView(int rows, int cols, CnMat* V, int r0, int c0) {
    if(rows != 0 && cols != 0) {
		cn_matrix_idx(V, r0, c0);
		cn_matrix_idx(V, r0 + rows - 1, c0 + cols - 1);
	}
	CnMat rtn = cnMat(rows, cols, cn_matrix_idx(V, r0, c0) + V->data);
	rtn.step = V->step;
	return rtn;
}

static inline CnMat cnMatConstView(int rows, int cols, const CnMat* V, int r0, int c0) {
    return cnMatView(rows, cols, (CnMat*)V, r0, c0);
}

/*
The function is a fast replacement for cvGetReal2D in the case of single-channel floating-point
matrices. It is faster because it is inline, it does fewer checks for array type and array element
type, and it checks for the row and column ranges only in debug mode.
@param mat Input matrix
@param row The zero-based index of row
@param col The zero-based index of column
 */
static inline FLT cnMatrixGet(const CnMat *mat, int row, int col) { return mat->data[cn_matrix_idx(mat, row, col)]; }

static inline FLT *cnMatrixPtr(const CnMat *mat, int row, int col) { return &mat->data[cn_matrix_idx(mat, row, col)]; }
/** @brief Sets a specific element of a single-channel floating-point matrix.

The function is a fast replacement for cvSetReal2D in the case of single-channel floating-point
matrices. It is faster because it is inline, it does fewer checks for array type and array element
type, and it checks for the row and column ranges only in debug mode.
@param mat The matrix
@param row The zero-based index of row
@param col The zero-based index of column
@param value The new value of the matrix element
 */
static inline void cnMatrixSet(CnMat *mat, int row, int col, FLT value) {
	mat->data[cn_matrix_idx(mat, row, col)] = value;
}
static inline void cnMatrixOptionalSet(CnMat *mat, int row, int col, FLT value) {
	if(row >= mat->rows || col >= mat->cols) return;
	cnMatrixSet(mat, row, col, value);
}

static inline void cn_get_diag(const struct CnMat *m, FLT *v, size_t cnt) {
	for (size_t i = 0; i < cnt; i++) {
		v[i] = cnMatrixGet(m, i, i);
	}
}
static inline void cn_set_diag(struct CnMat *m, const FLT *v) {
	for (int i = 0; i < m->rows; i++) {
		for (int j = 0; j < m->cols; j++) {
			cnMatrixSet(m, i, j, i == j ? (v ? v[i] : 1.) : 0.);
		}
	}
}

static inline bool cn_is_symmetrical(const struct CnMat *m) {
	if(m->rows != m->cols) return false;

	for (int i = 0; i < m->rows; i++) {
		for (int j = 0; j < i; j++) {
			if(cnMatrixGet(m, i, j) != cnMatrixGet(m, j, i))
				return false;
		}
	}
	return true;
}

static inline void cn_add_diag(struct CnMat *m, const CnMat* t, FLT scale) {
	assert(m->rows == m->cols);
	assert(m->rows == t->rows);
	assert(t->cols == 1);
	for (int i = 0; i < m->rows; i++) {
		cnMatrixSet(m, i, i, cnMatrixGet(m, i, i) + cn_as_const_vector(t)[i] * scale);
	}
}

CN_EXPORT_FUNCTION void cn_print_mat(const CnMat* M);
static inline void cn_set_diag_val(struct CnMat *m, FLT v) {
	for (int i = 0; i < m->rows; i++) {
		for (int j = 0; j < m->cols; j++) {
			cnMatrixSet(m, i, j, i == j ? v : 0.);
		}
	}
}

static inline void cn_eye(struct CnMat *m, const FLT *v) {
	for (int i = 0; i < m->rows; i++) {
		for (int j = 0; j < m->cols; j++) {
			cnMatrixSet(m, i, j, i == j ? (v ? v[i] : 1.) : 0.);
		}
	}
}

static inline void cn_copy_in_row_major_roi(struct CnMat *dst, const FLT *src, size_t src_stride, int start_i,
											int start_j, int end_i, int end_j) {
	for (int i = start_i; i < end_i; i++) {
		for (int j = start_j; j < end_j; j++) {
			cnMatrixSet(dst, i, j, src[j + i * src_stride]);
		}
	}
}

static inline void cn_copy_in_row_major(struct CnMat *dst, const FLT *src, size_t src_stride) {
	cn_copy_in_row_major_roi(dst, src, src_stride, 0, 0, dst->rows, dst->cols);
}
static inline FLT cn_sum(const struct CnMat *A) {
	FLT rtn = 0;
	for (int i = 0; i < A->rows; i++) {
		for (int j = 0; j < A->cols; j++) {
			rtn += cnMatrixGet(A, i, j);
		}
	}
	return rtn;
}
static inline FLT cn_trace(const struct CnMat *A) {
	FLT rtn = 0;
	int min_dim = A->rows;
	if (min_dim > A->cols)
		min_dim = A->cols;
	for (int i = 0; i < min_dim; i++) {
		for (int j = 0; j < min_dim; j++) {
			rtn += cnMatrixGet(A, i, j);
		}
	}

	return rtn;
}

static inline void cn_copy_data_in(struct CnMat *A, bool isRowMajor, const FLT *d) {
    assert(A && d);
#ifdef CN_MATRIX_IS_COL_MAJOR
	bool needsFlip = isRowMajor;
#else
	bool needsFlip = !isRowMajor;
#endif
	if (needsFlip) {
		CnMat t = cnMat(A->cols, A->rows, (FLT *)d);
		cnTranspose(&t, A);
	} else {
		memcpy(A->data, d, A->rows * A->cols * sizeof(FLT));
	}
}

static inline void cn_row_major_to_internal(struct CnMat *A, const FLT *d) { cn_copy_data_in(A, true, d); }

static inline void cn_col_major_to_internal(struct CnMat *A, const FLT *d) { cn_copy_data_in(A, false, d); }

static inline CnMat cnMat_from_row_major(int rows, int cols, FLT *data) {
	CnMat rtn = cnMat(rows, cols, data);
	cn_row_major_to_internal(&rtn, data);
	return rtn;
}

static inline CnMat cnMat_from_col_major(int rows, int cols, FLT *data) {
	CnMat rtn = cnMat(rows, cols, data);
	cn_col_major_to_internal(&rtn, data);
	return rtn;
}

static inline void cn_elementwise_subtract(struct CnMat *dst, const struct CnMat *A, const struct CnMat *B) {
    assert(dst->rows == A->rows && dst->cols == A->cols);
    assert(dst->rows == B->rows && dst->cols == B->cols);
	for (int i = 0; i < A->rows; i++) {
		for (int j = 0; j < A->cols; j++) {
			cnMatrixSet(dst, i, j, cnMatrixGet(A, i, j) - cnMatrixGet(B, i, j));
		}
	}
}
static inline void cn_elementwise_add(struct CnMat *dst, const struct CnMat *A, const struct CnMat *B) {
    assert(dst->rows == A->rows && dst->cols == A->cols);
    assert(dst->rows == B->rows && dst->cols == B->cols);
    for (int i = 0; i < A->rows; i++) {
        for (int j = 0; j < A->cols; j++) {
            cnMatrixSet(dst, i, j, cnMatrixGet(A, i, j) + cnMatrixGet(B, i, j));
        }
    }
}

static inline void cn_multiply_scalar(struct CnMat *dst, const struct CnMat *src, FLT scale) {
    assert(dst->rows == src->rows && dst->cols == src->cols);
	for(int i = 0;i < dst->cols * dst->rows;i++) dst->data[i] = src->data[i] * scale;
}

CN_EXPORT_FUNCTION void cn_ABAt_add(struct CnMat *out, const struct CnMat *A, const struct CnMat *B, const struct CnMat *C);
CN_EXPORT_FUNCTION const char* cnMatrixBackend();


static inline FLT cn_norm2(const struct CnMat *A) {
	FLT r = 0;
	for(int i = 0;i < A->cols * A->rows;i++) r += A->data[i] * A->data[i];
	return r;
}
#ifndef CN_MATRIX_IS_COL_MAJOR
static inline CnMat cn_row(struct CnMat *M, int r) {
	assert(r < M->rows);
	return cnMat(1, M->cols, M->data + M->step * r);
}
#endif

#define CNMATRIX_LOCAL_COPY(dst, src) \
{                            \
(dst) = *(src);                             \
(dst).step = (dst).cols;         \
(dst).data = (FLT*)alloca(sizeof(FLT) * (src)->rows * (src)->cols); \
cnCopy(src, &(dst), 0);      \
}\

#define CNMATRIX_LOCAL_COPY_IF_ALIAS(dst, src) if((dst).data == (src)->data) {CNMATRIX_LOCAL_COPY(dst, src);}

#ifdef __cplusplus
}

#include <vector>
static inline std::vector<FLT> cnMatToVector(const CnMat& m) {
    assert(m.cols == 1);
    std::vector<FLT> v;
    v.resize(m.rows);
    for(int i = 0;i < m.rows;i++)
        v[i] = m.data[i];
    return v;
}
static inline std::vector<std::vector<FLT>> cnMatToVectorVector(const CnMat& m) {
    std::vector<std::vector<FLT>> vv;
    vv.resize(m.rows);
    for(int i = 0;i < m.rows;i++) {
        vv[i].resize(m.cols);
        for (int j = 0; j < m.cols; j++) {
            vv[i][j] = cnMatrixGet(&m, i, j);
        }
    }
    return vv;
}
#define CNMATRIX_INCLUDED_FIRST
#include <cnmatrix/cn_matrix.hpp>

#endif
