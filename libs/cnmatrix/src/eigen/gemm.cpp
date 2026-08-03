#include "internal.h"

extern "C" void cnGEMM(const CnMat *_src1tmp, const CnMat *_src2tmp, double alpha, const CnMat *_src3tmp, double beta,
					   CnMat *_dst, enum cnGEMMFlags tABC) {
    CnMat _src1 = *_src1tmp;
    CnMat _src2 = *_src2tmp;
    CnMat _src3 = { };
	if (_src3tmp) {
        _src3 = *_src3tmp;
		//assert(_src3->data != _src2->data);
		//assert(_src3->data != _src1->data);
        CNMATRIX_LOCAL_COPY_IF_ALIAS(_src3, _dst);
		//assert(_src3->data != _dst->data);
	}
	//assert(_src2->data != _src1->data);
	//assert(_src2->data != _dst->data);
	//assert(_src1->data != _dst->data);

    CNMATRIX_LOCAL_COPY_IF_ALIAS(_src1, _dst);
    CNMATRIX_LOCAL_COPY_IF_ALIAS(_src2, _dst);

	int rows1 = (tABC & CN_GEMM_FLAG_A_T) ? _src1.cols : _src1.rows;
	int cols1 = (tABC & CN_GEMM_FLAG_A_T) ? _src1.rows : _src1.cols;

	int rows2 = (tABC & CN_GEMM_FLAG_B_T) ? _src2.cols : _src2.rows;
	int cols2 = (tABC & CN_GEMM_FLAG_B_T) ? _src2.rows : _src2.cols;

	if (_src3.data) {
		int rows3 = (tABC & CN_GEMM_FLAG_C_T) ? _src3.cols : _src3.rows;
		int cols3 = (tABC & CN_GEMM_FLAG_C_T) ? _src3.rows : _src3.cols;
		assert(rows3 == _dst->rows);
		assert(cols3 == _dst->cols);
	}

	// assert(src3 == 0 || beta != 0);
	assert(cols1 == rows2);
	assert(rows1 == _dst->rows);
	assert(cols2 == _dst->cols);

	auto src1 = CONVERT_TO_EIGEN(_src1);
	auto src2 = CONVERT_TO_EIGEN(_src2);

	auto dst = CONVERT_TO_EIGEN_PTR(_dst);

	EIGEN_RUNTIME_SET_IS_MALLOC_ALLOWED(false);
	if (tABC & CN_GEMM_FLAG_A_T)
		if (tABC & CN_GEMM_FLAG_B_T)
			dst.noalias() = alpha * src1.transpose() * src2.transpose();
		else
			dst.noalias() = alpha * src1.transpose() * src2;
	else {
		if (tABC & CN_GEMM_FLAG_B_T)
			dst.noalias() = alpha * src1 * src2.transpose();
		else
			dst.noalias() = alpha * src1 * src2;
	}

	if (_src3.data) {
		auto src3 = CONVERT_TO_EIGEN(_src3);
		if (tABC & CN_GEMM_FLAG_C_T)
			dst.noalias() += beta * src3.transpose();
		else
			dst.noalias() += beta * src3;
	}
	//assert(cn_is_finite(_dst));
}
