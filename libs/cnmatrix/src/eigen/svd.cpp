#include "internal.h"
#include <Eigen/LU>
#include <Eigen/SVD>
#include <Eigen/QR>


double cnInvert(const CnMat *srcarr, CnMat *dstarr, enum cnInvertMethod method) {
	auto src = CONVERT_TO_EIGEN_PTR(srcarr);
	auto dst = CONVERT_TO_EIGEN_PTR(dstarr);

	assert(srcarr->rows == dstarr->cols);
	assert(srcarr->cols == dstarr->rows);

	EIGEN_RUNTIME_SET_IS_MALLOC_ALLOWED(false);
	if (method == CN_INVERT_METHOD_LU) {
		assert(srcarr->rows == srcarr->cols);
		dst.noalias() = src.inverse();
	} else {
		dst.noalias() = src.completeOrthogonalDecomposition().pseudoInverse();
	}
	return 0;
}

extern "C" int cnSolve(const CnMat *_Aarr, const CnMat *_Barr, CnMat *_xarr, enum cnInvertMethod method) {
	auto Aarr = CONVERT_TO_EIGEN_PTR(_Aarr);
	auto Barr = CONVERT_TO_EIGEN_PTR(_Barr);
	auto xarr = CONVERT_TO_EIGEN_PTR(_xarr);

	if (method == CN_INVERT_METHOD_LU) {
		xarr.noalias() = Aarr.partialPivLu().solve(Barr);
	} else if (method == CN_INVERT_METHOD_QR) {
		xarr.noalias() = Aarr.colPivHouseholderQr().solve(Barr);
	} else {
		EIGEN_RUNTIME_SET_IS_MALLOC_ALLOWED(true);
		auto cnd = Aarr.jacobiSvd(
			Eigen::ComputeFullU |
			Eigen::ComputeFullV); 
		EIGEN_RUNTIME_SET_IS_MALLOC_ALLOWED(false);
		xarr.noalias() = cnd.solve(Barr);
	}
	return 0;
}

extern "C" void cnSVD(CnMat *aarr, CnMat *warr, CnMat *uarr, CnMat *varr, enum cnSVDFlags flags) {
	auto aarrEigen = CONVERT_TO_EIGEN_PTR(aarr);
	auto warrEigen = CONVERT_TO_EIGEN_PTR(warr);

	int options = 0;
	if (uarr)
		options |= Eigen::ComputeFullU;
	if (varr)
		options |= Eigen::ComputeFullV;
	EIGEN_RUNTIME_SET_IS_MALLOC_ALLOWED(true);
	auto cnd = aarrEigen.jacobiSvd(options);
	EIGEN_RUNTIME_SET_IS_MALLOC_ALLOWED(false);

	if (warrEigen.cols() == 1) {
		warrEigen.noalias() = cnd.singularValues();
	} else if (warrEigen.rows() == 1) {
		warrEigen.noalias() = cnd.singularValues().transpose();
	} else {
		warrEigen.diagonal().noalias() = cnd.singularValues();
	}

	if (uarr) {
		auto uarrEigen = CONVERT_TO_EIGEN_PTR(uarr);
		if (flags & CN_SVD_U_T)
			uarrEigen.noalias() = cnd.matrixU().transpose();
		else
			uarrEigen.noalias() = cnd.matrixU();
	}

	if (varr) {
		auto varrEigen = CONVERT_TO_EIGEN_PTR(varr);
		if (flags & CN_SVD_V_T)
			varrEigen.noalias() = cnd.matrixV().transpose();
		else
			varrEigen.noalias() = cnd.matrixV();
	}
}
