#pragma once

#define ASSERT_DOUBLE_ARRAY_EQ(n, arr1, arr2)                                                                          \
	for (int i = 0; i < n; i++) {                                                                                      \
		ASSERT_DOUBLE_EQ(arr1[i], arr2[i]);                                                                            \
	}

#define EXPECT_ARRAY_NEAR(n, arr1, arr2, abs_error)                                                                          \
	for (int i = 0; i < n; i++) {                                                                                      \
		EXPECT_NEAR(arr1[i], arr2[i], abs_error);                                                                            \
	}


static inline bool is_mat_near(const CnMat* x, const CnMat* y, float tol) {
    if(x->cols != y->cols) return false;
    if(x->rows != y->rows) return false;
    for(int i = 0;i < x->rows;i++) {
        for(int j = 0;j < x->cols;j++) {
            EXPECT_NEAR(cnMatrixGet(x, i, j), cnMatrixGet(y, i, j), tol);
            if(fabs(cnMatrixGet(x, i, j) - cnMatrixGet(y, i, j)) > tol) {
                return false;
            }
        }
    }
    return true;
}
#define EXPECT_MAT_NEAR(m1, m2) { \
                                  ASSERT_EQ(m1.rows == m2.rows);\
}
