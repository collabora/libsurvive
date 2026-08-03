#pragma once
#ifndef CNMATRIX_INCLUDED_FIRST
#error "Include <cnmatrix/cn_matrix.h>; not cn_matrix.hpp"
#endif
#include <cnmatrix/cn_matrix.h>
#include <memory>

namespace cnmatrix {
    struct Matrix {
        ::CnMat mat;
        std::shared_ptr<FLT> storage;
        Matrix(int rows, int cols = 1, FLT* data = 0) {
            if(data == 0) {
                storage = std::shared_ptr<FLT>((FLT *)calloc(rows * cols, sizeof(FLT)), free);
                data = storage.get();
            }
            mat = cnMat(rows, cols, data);
        }
        static Matrix Like(const Matrix& a) {
            return Matrix(a.mat.rows, a.mat.cols);
        }
        static Matrix Like(const CnMat & a) {
            return Matrix(a.rows, a.cols);
        }
        operator const CnMat*() const { return &mat; }
        operator CnMat*() { return &mat; }
        operator const CnMat&() const { return mat; }
        operator CnMat&() { return mat; }
        Matrix(const CnMat& cpy) : Matrix(cpy.rows, cpy.cols){
            cnCopy(&cpy, *this, 0);
        }
        FLT operator()(int i, int j) const { return mat(i, j);}
        FLT& operator()(int i, int j) { return mat(i,j); }
        FLT operator()(int i) const { return mat(i); }
        FLT& operator()(int i) { return mat(i); }
    };
}