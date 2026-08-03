#include "cnmatrix/cn_matrix.h"
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

void print_mat(const CnMat *M) {
	for (int i = 0; i < M->rows; i++) {
		for (int j = 0; j < M->cols; j++) {
			printf("%f,\t", cnMatrixGet(M, i, j));
		}
		printf("\n");
	}
	printf("\n");
}

bool assertFLTEquals(FLT a, FLT b) { return fabs(a - b) < 0.0001; }

int assertFLTAEquals(FLT *a, FLT *b, int length) {
	for (int i = 0; i < length; i++) {
		if (assertFLTEquals(a[i], b[i]) != true) {
			return i;
		}
	}
	return -1;
}

int checkMatFLTAEquals(const struct CnMat *a, const FLT *b) {
	for (int i = 0; i < a->rows; i++) {
		for (int j = 0; j < a->cols; j++) {
			if (assertFLTEquals(cnMatrixGet(a, i, j), b[i * a->cols + j]) == false)
				return i * a->cols + j;
		}
	}
	return -1;
}

#define PRINT_MAT(name)                                                                                                \
	printf(#name "\n");                                                                                                \
	print_mat(&name);

void test_gemm() {
	FLT _2x3[2 * 3] = {1, 2, 3, 4, 5, 6};
	CnMat m2x3 = cnMat_from_row_major(2, 3, _2x3);

	FLT _3x2[2 * 3] = {1, 3, 5, 2, 4, 6};
	CnMat m3x2 = cnMat_from_col_major(3, 2, _3x2);

	FLT _2x2[2 * 2] = {0};
	CnMat m2x2 = cnMat_from_row_major(2, 2, _2x2);

	FLT _3x3[3 * 3] = {0};
	CnMat m3x3 = cnMat_from_row_major(3, 3, _3x3);

	cnGEMM(&m2x3, &m3x2, 1, 0, 0, &m2x2, 0);
	cnGEMM(&m3x2, &m2x3, 1, 0, 0, &m3x3, 0);

	PRINT_MAT(m2x3);
	PRINT_MAT(m3x2);
	PRINT_MAT(m3x3);
	PRINT_MAT(m2x2);

	{
		FLT m2x2_gt[] = {
			22.000000,
			28.000000,
			49.000000,
			64.000000,
		};
		assert(checkMatFLTAEquals(&m2x2, m2x2_gt) < 0);
	}

	FLT m2x3_gt[] = {
		1.000000, 2.000000, 3.000000, 4.000000, 5.000000, 6.000000,
	};
	assert(checkMatFLTAEquals(&m2x3, m2x3_gt) < 0);

	cnGEMM(&m2x3, &m2x3, 1, 0, 0, &m3x3, CN_GEMM_FLAG_A_T);
	cnGEMM(&m2x3, &m2x3, 1, 0, 0, &m2x2, CN_GEMM_FLAG_B_T);
	PRINT_MAT(m3x3);
	PRINT_MAT(m2x2);

	{
		FLT m3x3_gt[] = {
			17.000000, 22.000000, 27.000000, 22.000000, 29.000000, 36.000000, 27.000000, 36.000000, 45.000000,
		};
		assert(checkMatFLTAEquals(&m3x3, m3x3_gt) < 0);
	}

	FLT m2x2_gt[] = {
		14.000000,
		32.000000,
		32.000000,
		77.000000,
	};
	assert(checkMatFLTAEquals(&m2x2, m2x2_gt) < 0);

	cnGEMM(&m2x3, &m3x2, 1, 0, 0, &m3x3, CN_GEMM_FLAG_A_T | CN_GEMM_FLAG_B_T);
	//  cvGEMM(&m3x2, &m2x3, 1, 0, 0, &m2x2, CN_GEMM_A_T | CN_GEMM_B_T);

	FLT m3x3_gt[] = {
		9.000000, 19.000000, 29.000000, 12.000000, 26.000000, 40.000000, 15.000000, 33.000000, 51.000000,
	};
	assert(checkMatFLTAEquals(&m3x3, m3x3_gt) < 0);
    PRINT_MAT(m3x3);

    FLT alias_gt[] = {1368,    2972,    4576, 2958,    6430,    9902, 4548,    9888,   15228};
    cnGEMM(&m3x3, &m3x3, 3, &m3x3, 2, &m3x3, CN_GEMM_FLAG_A_T);
    PRINT_MAT(m3x3);
    assert(checkMatFLTAEquals(&m3x3, alias_gt) < 0);

}

static void test_solve() {
	{
		FLT _A[3] = {1, 2, 3};
		FLT _B[3] = {4, 8, 12};
		FLT _x[1] = {0};

		CnMat A = cnMat_from_row_major(3, 1, _A);
		CnMat B = cnMat_from_row_major(3, 1, _B);
		CnMat x = cnMat_from_row_major(1, 1, _x);

		cnSolve(&A, &B, &x, CN_INVERT_METHOD_SVD);

		assert(fabs(_x[0] - 4) < .001);
	}

	{
		FLT _A[3] = {1, 2, 3};
		FLT _B[9] = {4, 5, 6, 7, 8, 9, 10, 11, 12};
		FLT _x[3] = {0};

		CnMat A = cnMat_from_row_major(3, 1, _A);
		CnMat B = cnMat_from_row_major(3, 3, _B);
		CnMat x = cnMat_from_row_major(1, 3, _x);

		cnSolve(&A, &B, &x, CN_INVERT_METHOD_SVD);
	}
}

static void test_invert() {
	printf("Invert:\n");

	FLT _3x3[3 * 3] = {1, 2, 3, 4, 5, 6, 7, 8, 12};
	CnMat m3x3 = cnMat_from_row_major(3, 3, _3x3);

	FLT _d3x3[3 * 3] = {0};
	CnMat d3x3 = cnMat_from_row_major(3, 3, _d3x3);

	FLT _i3x3[3 * 3] = {0};
	CnMat i3x3 = cnMat_from_row_major(3, 3, _i3x3);

	cnInvert(&m3x3, &d3x3, CN_INVERT_METHOD_LU);

	cnGEMM(&m3x3, &d3x3, 1, 0, 0, &i3x3, 0);
	print_mat(&d3x3);
	print_mat(&i3x3);

	FLT t = cn_trace(&i3x3);
	FLT s = cn_sum(&i3x3);

	assertFLTEquals(t, 3.);
	assertFLTEquals(s - t, 0);
}
static void test_invert2() {
    FLT _P[] = {+1.4192384e-03, +8.9898875e-06,              0, +9.7868476e-03, +4.2227069e-05,              0, +4.3565721e-10, +7.8105213e-09,
                +8.9898875e-06, +1.4821856e-03, +1.4099495e-09, +4.2254865e-05, +1.0083015e-02,              0,              0, +7.9221930e-06,
                0, +1.4099495e-09, +1.0021303e-07,              0, +2.1108031e-08, +9.9999912e-08,              0, +3.1559669e-07,
                +9.7868476e-03, +4.2254865e-05,              0, +1.4500066e-01, +2.9311173e-04,              0, +9.9479136e-08, +8.8850986e-08,
                +4.2227069e-05, +1.0083015e-02, +2.1108031e-08, +2.9311173e-04, +1.4712435e-01,              0,              0, +4.5619949e-04,
                0,              0, +9.9999912e-08,              0,              0, +1.0001000e-01,              0, +1.7626956e-10,
                +4.3565721e-10,              0,              0, +9.9479136e-08,              0,              0, +1.0009992e-02,              0,
                +7.8105213e-09, +7.9221930e-06, +3.1559669e-07, +8.8850986e-08, +4.5619949e-04, +1.7626956e-10,              0, +2.7158585e-02,};
    CnMat P = cnMat_from_row_major(8, 8, _P);
    CN_CREATE_STACK_MAT(iP, 8, 8);
    cnInvert(&P, &iP, CN_INVERT_METHOD_SVD);
    CN_CREATE_STACK_MAT(I, 8,8);
    cn_set_diag_val(&I, 1);

    CN_CREATE_STACK_MAT(should_be_I, 8, 8);
    cnGEMM(&P, &iP, 1, 0, 0, &should_be_I, 0);

    assert(checkMatFLTAEquals(&I, should_be_I.data));

    print_mat(&I);
    print_mat(&P);
    print_mat(&iP);
}
static void test_invert3() {
    FLT _P[] = {+1.4192384e-03, +8.9898875e-06,              0, +9.7868476e-03, +4.2227069e-05,              0, +4.3565721e-10, +7.8105213e-09,
                +8.9898875e-06, +1.4821856e-03, +1.4099495e-09, +4.2254865e-05, +1.0083015e-02,              0,              0, +7.9221930e-06,
                0, +1.4099495e-09, +1.0021303e-07,              0, +2.1108031e-08, +9.9999912e-08,              0, +3.1559669e-07,
                +9.7868476e-03, +4.2254865e-05,              0, +1.4500066e-01, +2.9311173e-04,              0, +9.9479136e-08, +8.8850986e-08,
                +4.2227069e-05, +1.0083015e-02, +2.1108031e-08, +2.9311173e-04, +1.4712435e-01,              0,              0, +4.5619949e-04,
                0,              0, +9.9999912e-08,              0,              0, +1.0001000e-01,              0, +1.7626956e-10,
                +4.3565721e-10,              0,              0, +9.9479136e-08,              0,              0, +1.0009992e-02,              0,
                +7.8105213e-09, +7.9221930e-06, +3.1559669e-07, +8.8850986e-08, +4.5619949e-04, +1.7626956e-10,              0, +2.7158585e-02,};

    CnMat LittleP = cnMat_from_row_major(8, 8, _P);
    CN_CREATE_STACK_MAT(BigP, 16, 16);
    CnMat P = cnMatView(8, 8, &BigP, 8, 8);
    cnCopy(&LittleP, &P, 0);
    assertFLTEquals(_P[0], cnMatrixGet(&BigP, 8, 8));

    CnMat iP = cnMatView(8, 8, &BigP, 0, 0);
    cnInvert(&P, &iP, CN_INVERT_METHOD_SVD);
    CN_CREATE_STACK_MAT(I, 8,8);
    cn_set_diag_val(&I, 1);

    CnMat should_be_I = cnMatView(8, 8, &BigP, 8, 0);
    cnGEMM(&P, &iP, 1, 0, 0, &should_be_I, 0);

    assert(checkMatFLTAEquals(&I, should_be_I.data));

    print_mat(&I);
    print_mat(&P);
    print_mat(&iP);
}
static void test_svd(CnMat* m3x3, CnMat*w,CnMat*u,CnMat*v) {
	printf("SVD:\n");

	cnSVD(m3x3, w, u, v, CN_SVD_U_T);

	PRINT_MAT(*w);
	PRINT_MAT(*u);
	PRINT_MAT(*v);

	CN_CREATE_STACK_MAT(fS, w->rows, w->rows);
	cn_set_diag(&fS, cn_as_const_vector(w));
	CN_CREATE_STACK_MAT(us, w->rows, w->rows);
	CN_CREATE_STACK_MAT(usvt, w->rows, w->rows);
	cnGEMM(u, &fS, 1, 0, 0, &us, CN_GEMM_FLAG_A_T);
	cnGEMM(&us, v, 1, 0, 0, &usvt, CN_GEMM_FLAG_B_T);

    print_mat(&us);
    print_mat(&usvt);
    assert(checkMatFLTAEquals(&usvt, m3x3->data));
	/*
	 * 0.16489968788789366, 5.6995693010571894e-05, -1.2222549954843702e-05,
  5.6995693010571894e-05, 0.035984545614046543, -5.7946162138306271e-05, -1.2222549954843702e-05,
  -5.7946162138306271e-05, 0.0046282902518506412
	 */
}
static void test_svd0() {
	FLT _3x3[3 * 3] = {1, 2, 3, 4, 5, 6, 7, 8, 12};
	CnMat m3x3 = cnMat_from_row_major(3, 3, _3x3);

	CN_CREATE_STACK_MAT(w, 3, 1);
	CN_CREATE_STACK_MAT(u, 3, 3);
	CN_CREATE_STACK_MAT(v, 3, 3);
	test_svd(&m3x3, &w, &u, &v);

	FLT wgt[] = {18.626945, 0.840495, 0.574865};
	assertFLTAEquals(cn_as_vector(&w), wgt, 3);
}
static void test_svd1() {
	FLT _3x3[3 * 3] = {0.16489968788789366, 5.6995693010571894e-05, -1.2222549954843702e-05,
					   5.6995693010571894e-05, 0.035984545614046543, -5.7946162138306271e-05, -1.2222549954843702e-05,
					   -5.7946162138306271e-05, 0.0046282902518506412};
    CnMat m3x3 = cnMat_from_row_major(3, 3, _3x3);

	CN_CREATE_STACK_MAT(w, 3, 1);
	CN_CREATE_STACK_MAT(u, 3, 3);
	CN_CREATE_STACK_MAT(v, 3, 3);
	test_svd(&m3x3, &w, &u, &v);


	FLT wgt[] = {0.16489971402272774, 0.035984627479101181, 0.0046281822519620109};
	assertFLTAEquals(cn_as_vector(&w), wgt, 3);
}
static void test_svd2() {
    FLT _P[] = {+1.4192384e-03, +8.9898875e-06,              0, +9.7868476e-03, +4.2227069e-05,              0, +4.3565721e-10, +7.8105213e-09,
                +8.9898875e-06, +1.4821856e-03, +1.4099495e-09, +4.2254865e-05, +1.0083015e-02,              0,              0, +7.9221930e-06,
                0, +1.4099495e-09, +1.0021303e-07,              0, +2.1108031e-08, +9.9999912e-08,              0, +3.1559669e-07,
                +9.7868476e-03, +4.2254865e-05,              0, +1.4500066e-01, +2.9311173e-04,              0, +9.9479136e-08, +8.8850986e-08,
                +4.2227069e-05, +1.0083015e-02, +2.1108031e-08, +2.9311173e-04, +1.4712435e-01,              0,              0, +4.5619949e-04,
                0,              0, +9.9999912e-08,              0,              0, +1.0001000e-01,              0, +1.7626956e-10,
                +4.3565721e-10,              0,              0, +9.9479136e-08,              0,              0, +1.0009992e-02,              0,
                +7.8105213e-09, +7.9221930e-06, +3.1559669e-07, +8.8850986e-08, +4.5619949e-04, +1.7626956e-10,              0, +2.7158585e-02};
    CnMat P = cnMat_from_row_major(8, 8, _P);


    CN_CREATE_STACK_MAT(w, 8, 1);
    CN_CREATE_STACK_MAT(u, 8, 8);
    CN_CREATE_STACK_MAT(v, 8, 8);
    test_svd(&P, &w, &u, &v);

    FLT wgt[] = {1.4786e-01,
            1.4562e-01,
            1.0001e-01,
            2.7157e-02,
            1.0010e-02,
            7.8805e-04,
            1.0021e-07};
    assertFLTAEquals(cn_as_vector(&w), wgt, 8);
}
static void test_multrans() {
	FLT _A[3] = {1, 2, 3};
	CnMat A = cnMat(3, 1, _A);

	FLT _B[9];
	CnMat B = cnMat(3, 3, _B);

	FLT _C[1];
	CnMat C = cnMat(1, 1, _C);

	cnMulTransposed(&A, &B, 0, 0, 1);
	PRINT_MAT(B);
	FLT ans[] = {1, 2, 3, 2, 4, 6, 3, 6, 9};
	for (int i = 0; i < 9; i++) {
		assert(ans[i] == _B[i]);
	}

	cnMulTransposed(&A, &C, 1, 0, 10);
	PRINT_MAT(C);
	assert(_C[0] == 140);
}

static void test_sym_sqrt() {
    FLT _A[3 * 3] = {
            4, 12, -16,
            12, 37, -43,
            -16, -43, 98
    };
    CnMat A = cnMat_from_row_major(3, 3, _A);

    FLT _B[3 * 3] = {
            2, 0, 0,
            6, 1, 0,
            -8, 5, 3
    };
    CnMat B = cnMat_from_row_major(3, 3, _B);

    CN_CREATE_STACK_MAT(C, 3, 3);

    cnSqRootSymmetric(&A, &C);
    assertFLTAEquals(_B, _C, 3 * 3);
}

int main()
{
    test_svd2();
	test_invert();
    test_invert2();
    test_invert3();
	test_gemm();
	test_solve();
	test_svd0();
	test_svd1();

	test_multrans();
    test_sym_sqrt();
	return 0;
}

