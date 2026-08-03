//#define EIGEN_RUNTIME_NO_MALLOC
#include "internal.h"
const char* cnMatrixBackend() {
	return "Eigen";
}

void cnSqRootSymmetric(const CnMat *srcarr, CnMat *dstarr) {
    auto src = CONVERT_TO_EIGEN_PTR(srcarr);
    auto dst = CONVERT_TO_EIGEN_PTR(dstarr);

    EIGEN_RUNTIME_SET_IS_MALLOC_ALLOWED(false);
    dst.noalias() = Eigen::LLT<MatrixType>(src).matrixL().toDenseMatrix();
}

void cnMulTransposed(const CnMat *src, CnMat *dst, int order, const CnMat *delta, double scale) {
	auto srcEigen = CONVERT_TO_EIGEN_PTR(src);
	auto dstEigen = CONVERT_TO_EIGEN_PTR(dst);

	int drows = order == 0 ? dst->rows : dst->cols;
	assert(drows == dst->cols);
	assert(order == 1 ? (dst->cols == src->cols) : (dst->cols == src->rows));

	if (delta) {
		auto deltaEigen = CONVERT_TO_EIGEN_PTR(delta);
		if (order == 0)
			dstEigen.noalias() = scale * (srcEigen - deltaEigen) * (srcEigen - deltaEigen).transpose();
		else
			dstEigen.noalias() = scale * (srcEigen - deltaEigen).transpose() * (src - delta);
	} else {
		if (order == 0)
			dstEigen.noalias() = scale * srcEigen * srcEigen.transpose();
		else
			dstEigen.noalias() = scale * srcEigen.transpose() * srcEigen;
	}
}

void cnTranspose(const CnMat *M, CnMat *dst) {
	auto src = CONVERT_TO_EIGEN_PTR(M);
	auto dstEigen = CONVERT_TO_EIGEN_PTR(dst);
	if (CN_FLT_PTR(M) == CN_FLT_PTR(dst))
		dstEigen = src.transpose().eval();
	else
		dstEigen.noalias() = src.transpose();
}

void print_mat(const CnMat *M);

double cnDet(const CnMat *M) {
	EIGEN_RUNTIME_SET_IS_MALLOC_ALLOWED(false);
	auto MEigen = CONVERT_TO_EIGEN_PTR(M);
	return MEigen.determinant();
}

