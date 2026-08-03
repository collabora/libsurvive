#include <cblas.h>
#ifdef LAPACKE_FOLDER
#include <lapacke/lapacke.h>
#include <lapacke/lapacke_utils.h>
#else
#include <lapacke.h>
#include <lapacke_utils.h>
#endif

#include "math.h"
#include "stdbool.h"
#include "stdio.h"
#include "string.h"
#include "cnmatrix/cn_matrix.h"

#include <limits.h>
#include <stdarg.h>

#ifdef _WIN32
#define SURVIVE_LOCAL_ONLY
#include <malloc.h>
#define alloca _alloca
#else
#define SURVIVE_LOCAL_ONLY __attribute__((visibility("hidden")))
#endif

#define CN_Error(code, msg) assert(0 && msg); // cv::error( code, msg, CN_Func, __FILE__, __LINE__ )

const int DECOMP_SVD = 1;
const int DECOMP_LU = 2;

void print_mat(const CnMat *M);

static size_t mat_size_bytes(const CnMat *mat) { return (size_t)sizeof(FLT) * mat->cols * mat->rows; }

#ifdef CN_USE_FLOAT
#define cblas_gemm cblas_sgemm
#define cblas_symm cblas_ssymm
#define LAPACKE_getrs LAPACKE_sgetrs
#define LAPACKE_getrf LAPACKE_sgetrf
#define LAPACKE_getri LAPACKE_sgetri
#define LAPACKE_gelss LAPACKE_sgelss
#define LAPACKE_gesvd LAPACKE_sgesvd
#define LAPACKE_gesvd_work LAPACKE_sgesvd_work
#define LAPACKE_getri_work LAPACKE_sgetri_work
#define LAPACKE_ge_trans LAPACKE_sge_trans
#define LAPACKE_potrf LAPACKE_spotrf
#else
#define cblas_gemm cblas_dgemm
#define cblas_symm cblas_dsymm
#define LAPACKE_getrs LAPACKE_dgetrs
#define LAPACKE_getrf LAPACKE_dgetrf
#define LAPACKE_getri LAPACKE_dgetri
#define LAPACKE_gelss LAPACKE_dgelss
#define LAPACKE_gesvd LAPACKE_dgesvd
#define LAPACKE_gesvd_work LAPACKE_dgesvd_work
#define LAPACKE_getri_work LAPACKE_dgetri_work
#define LAPACKE_ge_trans LAPACKE_dge_trans
    #define LAPACKE_potrf LAPACKE_dpotrf
#endif

// dst = alpha * src1 * src2 + beta * src3 or dst = alpha * src2 * src1 + beta * src3 where src1 is symm
SURVIVE_LOCAL_ONLY void cnSYMM(const CnMat *src1, const CnMat *src2, double alpha, const CnMat *src3, double beta,
							   CnMat *dst, bool src1First) {

	int rows1 = src1->rows;
	int cols1 = src1->cols;

	int rows2 = src2->rows;
	int cols2 = src2->cols;

	if (src3) {
		int rows3 = src3->rows;
		int cols3 = src3->cols;
		assert(rows3 == dst->rows);
		assert(cols3 == dst->cols);
	}

	// assert(src3 == 0 || beta != 0);
	assert(cols1 == rows2);
	assert(rows1 == dst->rows);
	assert(cols2 == dst->cols);

	lapack_int lda = src1->cols;
	lapack_int ldb = src2->cols;

	if (src3)
		cnCopy(src3, dst, 0);
	else
		beta = 0;

	assert(CN_RAW_PTR(dst) != CN_RAW_PTR(src1));
	assert(CN_RAW_PTR(dst) != CN_RAW_PTR(src2));
	/*
		void cblas_dsymm(OPENBLAS_CONST enum CBLAS_ORDER Order,
						 OPENBLAS_CONST enum CBLAS_SIDE Side,
						 OPENBLAS_CONST enum CBLAS_UPLO Uplo,
						 OPENBLAS_CONST blasint M,
						 OPENBLAS_CONST blasint N,
						 OPENBLAS_CONST double alpha,
						 OPENBLAS_CONST double *A,
						 OPENBLAS_CONST blasint lda,
						 OPENBLAS_CONST double *B,
						 OPENBLAS_CONST blasint ldb,
						 OPENBLAS_CONST double beta,
						 double *C,
						 OPENBLAS_CONST blasint ldc);
	*/
	cblas_symm(CblasRowMajor, src1First ? CblasLeft : CblasRight, CblasUpper, dst->rows, dst->cols, alpha,
			   CN_RAW_PTR(src1), lda, CN_RAW_PTR(src2), ldb, beta, CN_RAW_PTR(dst), dst->cols);
}

// Special case dst = alpha * src2 * src1 * src2' + beta * src3
void mulBABt(const CnMat *src1, const CnMat *src2, double alpha, const CnMat *src3, double beta, CnMat *dst) {
	size_t dims = src2->rows;
	assert(src2->cols == src2->rows);
	CN_CREATE_STACK_MAT(tmp, dims, dims);

	// This has been profiled; and weirdly enough the SYMM version is slower for a 19x19 matrix. Guessing access order
	// or some other cache thing matters more than the additional 2x multiplications.
//#define USE_SYM
#ifdef USE_SYM
	cnSYMM(src1, src2, 1, 0, 0, &tmp, false);
	cnGEMM(&tmp, src2, alpha, src3, beta, dst, CN_GEMM_B_T);
#else
	cnGEMM(src1, src2, 1, 0, 0, &tmp, CN_GEMM_FLAG_B_T);
	cnGEMM(src2, &tmp, alpha, src3, beta, dst, 0);
#endif
	CN_FREE_STACK_MAT(tmp);
}

// dst = alpha * src1 * src2 + beta * src3
SURVIVE_LOCAL_ONLY void cnGEMM(const CnMat *src1, const CnMat *src2, double alpha, const CnMat *src3, double beta,
							   CnMat *dst, enum cnGEMMFlags tABC) {

	int rows1 = (tABC & CN_GEMM_FLAG_A_T) ? src1->cols : src1->rows;
	int cols1 = (tABC & CN_GEMM_FLAG_A_T) ? src1->rows : src1->cols;

	int rows2 = (tABC & CN_GEMM_FLAG_B_T) ? src2->cols : src2->rows;
	int cols2 = (tABC & CN_GEMM_FLAG_B_T) ? src2->rows : src2->cols;

	CnMat src1_local = *src1;
	CnMat src2_local = *src2;

	CNMATRIX_LOCAL_COPY_IF_ALIAS(src1_local, src1);
	CNMATRIX_LOCAL_COPY_IF_ALIAS(src2_local, src2);

	if (src3) {
		int rows3 = (tABC & CN_GEMM_FLAG_C_T) ? src3->cols : src3->rows;
		int cols3 = (tABC & CN_GEMM_FLAG_C_T) ? src3->rows : src3->cols;
		assert(rows3 == dst->rows);
		assert(cols3 == dst->cols);
	}

	// assert(src3 == 0 || beta != 0);
	assert(cols1 == rows2);
	assert(rows1 == dst->rows);
	assert(cols2 == dst->cols);

	if(dst->rows == 0 || dst->cols == 0)
		return;

	lapack_int lda = src1_local.step;
	lapack_int ldb = src2_local.step;

	if (src3)
		cnCopy(src3, dst, 0);
	else
		beta = 0;

	assert(dst->cols > 0);
	cblas_gemm(CblasRowMajor, (tABC & CN_GEMM_FLAG_A_T) ? CblasTrans : CblasNoTrans,
			   (tABC & CN_GEMM_FLAG_B_T) ? CblasTrans : CblasNoTrans, dst->rows, dst->cols, cols1, alpha,
			   CN_RAW_PTR(&src1_local), lda, CN_RAW_PTR(&src2_local), ldb, beta, CN_RAW_PTR(dst), dst->step);
}

// dst = scale * src ^ t * src     iff order == 1
// dst = scale *     src * src ^ t iff order == 0
SURVIVE_LOCAL_ONLY void cnMulTransposed(const CnMat *src, CnMat *dst, int order, const CnMat *delta, double scale) {
	lapack_int rows = src->rows;
	lapack_int cols = src->cols;

	lapack_int drows = order == 0 ? dst->rows : dst->cols;
	assert(drows == dst->cols);
	assert(order == 1 ? (dst->cols == src->cols) : (dst->cols == src->rows));
	assert(delta == 0 && "This isn't implemented yet");
	double beta = 0;

	bool isAT = order == 1;
	bool isBT = !isAT;

	lapack_int dstCols = dst->cols;
	assert(dstCols > 0);
	cblas_gemm(CblasRowMajor, isAT ? CblasTrans : CblasNoTrans, isBT ? CblasTrans : CblasNoTrans, dst->rows, dst->cols,
			   order == 1 ? src->rows : src->cols, scale, CN_RAW_PTR(src), src->cols, CN_RAW_PTR(src), src->cols, beta,
			   CN_RAW_PTR(dst), dstCols);
}

/* IEEE754 constants and macros */
#define CN_TOGGLE_FLT(x) ((x) ^ ((int)(x) < 0 ? 0x7fffffff : 0))
#define CN_TOGGLE_DBL(x) ((x) ^ ((int64)(x) < 0 ? CN_BIG_INT(0x7fffffffffffffff) : 0))

#define CN_DbgAssert assert

#define CN_CREATE_MAT_HEADER_ALLOCA(stack_mat, rows, cols)                                                             \
	CnMat *stack_mat = cnInitMatHeader(CN_MATRIX_ALLOC(sizeof(CnMat)), rows, cols);

#define CN_CREATE_MAT_ALLOCA(stack_mat, height, width)                                                                 \
	CN_CREATE_MAT_HEADER_ALLOCA(stack_mat, height, width);                                                             \
	(stack_mat)->data = CN_MATRIX_ALLOC(mat_size_bytes(stack_mat));

#define CN_MAT_ALLOCA_FREE(stack_mat)                                                                                  \
	{                                                                                                                  \
		CN_MATRIX_FREE(stack_mat->data);                                                                               \
		CN_MATRIX_FREE(stack_mat);                                                                                     \
	}

#define CREATE_CN_STACK_MAT(name, rows, cols, type)                                                                    \
	FLT *_##name = alloca(rows * cols * sizeof(FLT));                                                                  \
	CnMat name = cnMat(rows, cols, _##name);

static lapack_int LAPACKE_gesvd_static_alloc(int matrix_layout, char jobu, char jobvt, lapack_int m, lapack_int n,
											 FLT *a, lapack_int lda, FLT *s, FLT *u, lapack_int ldu, FLT *vt,
											 lapack_int ldvt, FLT *superb) {
	lapack_int info = 0;
	lapack_int lwork = -1;
	FLT *work = NULL;
	FLT work_query;
	lapack_int i;
	if (matrix_layout != LAPACK_COL_MAJOR && matrix_layout != LAPACK_ROW_MAJOR) {
		LAPACKE_xerbla("LAPACKE_dgesvd", -1);
		return -1;
	}

	/* Query optimal working array(s) size */
	info = LAPACKE_gesvd_work(matrix_layout, jobu, jobvt, m, n, a, lda, s, u, ldu, vt, ldvt, &work_query, lwork);
	if (info != 0) {
		goto exit_level_0;
	}
	lwork = (lapack_int)work_query;
	/* Allocate memory for work arrays */
	work = (FLT *)CN_MATRIX_ALLOC(sizeof(FLT) * lwork);
	memset(work, 0, sizeof(FLT) * lwork);

	if (work == NULL) {
		info = LAPACK_WORK_MEMORY_ERROR;
		goto exit_level_0;
	}
	/* Call middle-level interface */
	info = LAPACKE_gesvd_work(matrix_layout, jobu, jobvt, m, n, a, lda, s, u, ldu, vt, ldvt, work, lwork);
	/* Backup significant data from working array(s) */
	for (i = 0; i < MIN(m, n) - 1; i++) {
		superb[i] = work[i + 1];
	}
	CN_MATRIX_FREE(work);

exit_level_0:
	if (info == LAPACK_WORK_MEMORY_ERROR) {
		LAPACKE_xerbla("LAPACKE_dgesvd", info);
	}
	return info;
}

static inline lapack_int LAPACKE_getri_static_alloc(int matrix_layout, lapack_int n, FLT *a, lapack_int lda,
													const lapack_int *ipiv) {
	lapack_int info = 0;
	lapack_int lwork = -1;
	FLT *work = NULL;
	FLT work_query;
	if (matrix_layout != LAPACK_COL_MAJOR && matrix_layout != LAPACK_ROW_MAJOR) {
		LAPACKE_xerbla("LAPACKE_dgetri", -1);
		return -1;
	}
	/* Query optimal working array(s) size */
	info = LAPACKE_getri_work(matrix_layout, n, a, lda, ipiv, &work_query, lwork);
	if (info != 0) {
		goto exit_level_0;
	}
	lwork = (lapack_int)work_query;
	/* Allocate memory for work arrays */
	work = (FLT *)alloca(sizeof(FLT) * lwork);
	memset(work, 0, sizeof(FLT) * lwork);
	if (work == NULL) {
		info = LAPACK_WORK_MEMORY_ERROR;
		goto exit_level_0;
	}
	/* Call middle-level interface */
	info = LAPACKE_getri_work(matrix_layout, n, a, lda, ipiv, work, lwork);
	/* Release memory and exit */

exit_level_0:
	if (info == LAPACK_WORK_MEMORY_ERROR) {
		LAPACKE_xerbla("LAPACKE_dgetri", info);
	}
	return info;
}

SURVIVE_LOCAL_ONLY double cnInvert(const CnMat *srcarr, CnMat *dstarr, enum cnInvertMethod method) {
	lapack_int inf;
	lapack_int rows = srcarr->rows;
	lapack_int cols = srcarr->cols;
	lapack_int lda = srcarr->step;

	cnCopy(srcarr, dstarr, 0);
	FLT *a = CN_RAW_PTR(dstarr);

#ifdef DEBUG_PRINT
	printf("a: \n");
	print_mat(srcarr);
#endif
	if (method == CN_INVERT_METHOD_LU) {
		lapack_int *ipiv = CN_MATRIX_ALLOC(sizeof(lapack_int) * MIN(srcarr->rows, srcarr->cols));

		lapack_int lda_t = MAX(1, rows);

		FLT *a_t = (FLT *)CN_MATRIX_ALLOC(sizeof(FLT) * lda_t * MAX(1, cols));
		LAPACKE_ge_trans(LAPACK_ROW_MAJOR, rows, cols, a, lda, a_t, lda_t);

		inf = LAPACKE_getrf(LAPACK_COL_MAJOR, rows, cols, a_t, lda, ipiv);
		assert(inf == 0);

		inf = LAPACKE_getri_static_alloc(LAPACK_COL_MAJOR, rows, a_t, lda, ipiv);
		assert(inf >= 0);

		LAPACKE_ge_trans(LAPACK_COL_MAJOR, rows, cols, a_t, lda, a, lda_t);

		if (inf > 0) {
			printf("Warning: Singular matrix: \n");
			// print_mat(srcarr);
		}

		CN_MATRIX_FREE(a_t);
		CN_MATRIX_FREE(ipiv);
		// free(ipiv);

	} else if (method == DECOMP_SVD) {
		CN_CREATE_STACK_MAT(w, 1, MIN(dstarr->rows, dstarr->cols));
		CN_CREATE_STACK_MAT(u, dstarr->cols, dstarr->cols);
		CN_CREATE_STACK_MAT(v, dstarr->rows, dstarr->rows);
		CN_CREATE_STACK_MAT(um, w.cols, w.cols);

		cnSVD(dstarr, &w, &u, &v, 0);

		cnSetZero(&um);
		for (int i = 0; i < w.cols; i++) {
			if (_w[i] != 0.0)
				cnMatrixSet(&um, i, i, 1. / (_w)[i]);
		}

        CN_CREATE_STACK_MAT(tmp, dstarr->cols, dstarr->rows);

		cnGEMM(&v, &um, 1, 0, 0, &tmp, 0);
		cnGEMM(&tmp, &u, 1, 0, 0, dstarr, CN_GEMM_FLAG_B_T);

		CN_FREE_STACK_MAT(um);
		CN_FREE_STACK_MAT(v);
		CN_FREE_STACK_MAT(u);
		CN_FREE_STACK_MAT(w);
	} else {
		assert(0 && "Bad argument");
		return -1;
	}

	return 0;
}

#define CN_CLONE_MAT_ALLOCA(stack_mat, mat)                                                                            \
	CN_CREATE_MAT_ALLOCA(stack_mat, mat->rows, mat->cols)                                                              \
	cnCopy(mat, stack_mat, 0);

static int cnSolve_LU(const CnMat *Aarr, const CnMat *Barr, CnMat *xarr) {
	lapack_int inf;
	lapack_int arows = Aarr->rows;
	lapack_int acols = Aarr->cols;
	lapack_int xcols = Barr->cols;
	lapack_int xrows = Barr->rows;
	lapack_int lda = acols; // Aarr->step / sizeof(double);

	assert(Aarr->cols == xarr->rows);
	assert(Barr->rows == Aarr->rows);
	assert(xarr->cols == Barr->cols);

	cnCopy(Barr, xarr, 0);
	FLT *a_ws = CN_MATRIX_ALLOC(mat_size_bytes(Aarr));
	memcpy(a_ws, CN_RAW_PTR(Aarr), mat_size_bytes(Aarr));

	lapack_int brows = xarr->rows;
	lapack_int bcols = xarr->cols;
	lapack_int ldb = bcols; // Barr->step / sizeof(double);

	lapack_int *ipiv = CN_MATRIX_ALLOC(sizeof(lapack_int) * MIN(Aarr->rows, Aarr->cols));

	inf = LAPACKE_getrf(LAPACK_ROW_MAJOR, arows, acols, (a_ws), lda, ipiv);
	assert(inf >= 0);
	if (inf > 0) {
		printf("Warning: Singular matrix: \n");
		// print_mat(a_ws);
	}

#ifdef DEBUG_PRINT
	printf("Solve A * x = B:\n");
	// print_mat(a_ws);
	print_mat(Barr);
#endif

	inf = LAPACKE_getrs(LAPACK_ROW_MAJOR, CblasNoTrans, arows, bcols, (a_ws), lda, ipiv, CN_RAW_PTR(xarr), ldb);
	assert(inf == 0);

	CN_MATRIX_FREE(a_ws);
	CN_MATRIX_FREE(ipiv);
	return 0;
}

void cnSqRootSymmetric(const CnMat *srcarr, CnMat *dstarr) {
    assert(srcarr->rows == srcarr->cols);
    assert(dstarr->rows == dstarr->cols);
    assert(dstarr->rows == srcarr->cols);

    cnCopy(srcarr, dstarr, 0);
    int info = LAPACKE_potrf(LAPACK_ROW_MAJOR, 'L', srcarr->cols, dstarr->data, dstarr->step);
    for(int i = 0;i < dstarr->cols;i++) {
        for(int j = i + 1;j < dstarr->cols;j++) {
            cnMatrixSet(dstarr, i, j, 0);
        }
    }
    assert(info >= 0);
}

static inline int cnSolve_SVD(const CnMat *Aarr, const CnMat *Barr, CnMat *xarr) {
	lapack_int arows = Aarr->rows;
	lapack_int acols = Aarr->cols;
	lapack_int xcols = Barr->cols;

	bool xLargerThanB = Barr->rows > acols;
	CnMat *xCpy = 0;
	if (xLargerThanB) {
		CN_CLONE_MAT_ALLOCA(xCpyStack, Barr);
		xCpy = xCpyStack;
	} else {
		xCpy = xarr;
		memcpy(CN_RAW_PTR(xarr), CN_RAW_PTR(Barr), mat_size_bytes(Barr));
	}

	// CnMat *aCpy = cnCloneMat(Aarr);
	FLT *aCpy = CN_MATRIX_ALLOC(mat_size_bytes(Aarr));
	memcpy(aCpy, CN_RAW_PTR(Aarr), mat_size_bytes(Aarr));

	FLT *S = CN_MATRIX_ALLOC(sizeof(FLT) * MIN(arows, acols));
	// FLT *S = malloc(sizeof(FLT) * MIN(arows, acols));
	FLT rcond = -1;
	lapack_int *rank = CN_MATRIX_ALLOC(sizeof(lapack_int) * MIN(arows, acols));
	lapack_int inf =
		LAPACKE_gelss(LAPACK_ROW_MAJOR, arows, acols, xcols, (aCpy), acols, CN_RAW_PTR(xCpy), xcols, S, rcond, rank);
	assert(xarr->rows == acols);
	assert(xarr->cols == xCpy->cols);

	if (xLargerThanB) {
		xCpy->rows = acols;
		cnCopy(xCpy, xarr, 0);
		// cnReleaseMat(&xCpy);
	}

	CN_MATRIX_FREE(rank);
	CN_MATRIX_FREE(aCpy);
	CN_MATRIX_FREE(S);
	if (xLargerThanB) {
		CN_MAT_ALLOCA_FREE(xCpy);
	}

	assert(inf == 0);
	if (inf != 0)
		return inf;
	return 0;
}

SURVIVE_LOCAL_ONLY int cnSolve(const CnMat *Aarr, const CnMat *Barr, CnMat *xarr, enum cnInvertMethod method) {
	if (method == CN_INVERT_METHOD_LU) {
		return cnSolve_LU(Aarr, Barr, xarr);
	} else if (method == CN_INVERT_METHOD_SVD || method == CN_INVERT_METHOD_QR) {
		return cnSolve_SVD(Aarr, Barr, xarr);
	} else {
		assert("Unknown method to solve" && 0);
	}
	return -1;
}

SURVIVE_LOCAL_ONLY void cnTranspose(const CnMat *M, CnMat *dst) {
	bool inPlace = M == dst || CN_RAW_PTR(M) == CN_RAW_PTR(dst);
	FLT *src = CN_RAW_PTR(M);

	if (inPlace) {
		src = alloca(mat_size_bytes(M));
		memcpy(src, CN_RAW_PTR(M), mat_size_bytes(M));
	} else {
		assert(M->rows == dst->cols);
		assert(M->cols == dst->rows);
	}

	for (unsigned i = 0; i < M->rows; i++) {
		for (unsigned j = 0; j < M->cols; j++) {
			CN_RAW_PTR(dst)[j * M->rows + i] = src[i * M->cols + j];
		}
	}
}

#if defined(__has_feature)
#if __has_feature(memory_sanitizer)
#define MEMORY_SANITIZER_IGNORE __attribute__((no_sanitize("memory")))
#endif
#endif
#ifndef MEMORY_SANITIZER_IGNORE
#define MEMORY_SANITIZER_IGNORE
#endif

#define CALLOCA(size) memset(alloca(size), 0, size)

SURVIVE_LOCAL_ONLY void cnSVD(CnMat *aarr, CnMat *warr, CnMat *uarr, CnMat *varr, enum cnSVDFlags flags) {
	char jobu = 'A';
	char jobvt = 'A';

	lapack_int inf;

	if ((flags & CN_SVD_MODIFY_A) == 0) {
		aarr = cnCloneMat(aarr);
	}

	if (uarr == 0)
		jobu = 'N';
	if (varr == 0)
		jobvt = 'N';

	lapack_int arows = aarr->rows, acols = aarr->cols;

	FLT *pw = warr ? CN_RAW_PTR(warr) : (FLT *)CALLOCA(sizeof(FLT) * arows * acols);
	FLT *pu = uarr ? CN_RAW_PTR(uarr) : (FLT *)CALLOCA(sizeof(FLT) * arows * arows);
	FLT *pv = varr ? CN_RAW_PTR(varr) : (FLT *)CALLOCA(sizeof(FLT) * acols * acols);

	lapack_int ulda = uarr ? uarr->step : aarr->step;
	lapack_int plda = varr ? varr->step : aarr->step;

	FLT *superb = CALLOCA(sizeof(FLT) * MIN(arows, acols));
	inf = LAPACKE_gesvd_static_alloc(LAPACK_ROW_MAJOR, jobu, jobvt, arows, acols, CN_RAW_PTR(aarr), acols, pw, pu, ulda,
									 pv, plda, superb);

	switch (inf) {
	case -6:
		assert(false && "matrix has NaNs");
		break;
	case 0:
		break;
	default:
		assert(inf == 0);
	}

	if (uarr && (flags & CN_SVD_U_T)) {
		cnTranspose(uarr, uarr);
	}

	if (varr && (flags & CN_SVD_V_T) == 0) {
		cnTranspose(varr, varr);
	}

	if ((flags & CN_SVD_MODIFY_A) == 0) {
	    free(aarr->data);
		cnReleaseMat(&aarr);
	}
}

const char* cnMatrixBackend() {
	return "BLAS";
}

SURVIVE_LOCAL_ONLY double cnDet(const CnMat *M) {
	assert(M->rows == M->cols);
	assert(M->rows <= 3 && "cnDet unimplemented for matrices >3");

	FLT *m = CN_RAW_PTR(M);

	switch (M->rows) {
	case 1:
		return m[0];
	case 2: {
		return m[0] * m[3] - m[1] * m[2];
	}
	case 3: {
		FLT m00 = m[0], m01 = m[1], m02 = m[2], m10 = m[3], m11 = m[4], m12 = m[5], m20 = m[6], m21 = m[7], m22 = m[8];

		return m00 * (m11 * m22 - m12 * m21) - m01 * (m10 * m22 - m12 * m20) + m02 * (m10 * m21 - m11 * m20);
	}
	default:
		abort();
	}
}

// https://fossies.org/linux/OpenBLAS/USAGE.md
extern void __attribute__ ((__weak__)) openblas_set_num_threads(int);
void __attribute__((constructor)) force_thread_count() {
  if(openblas_set_num_threads) {
    openblas_set_num_threads(1);
  }
}
