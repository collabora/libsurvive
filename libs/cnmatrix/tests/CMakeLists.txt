include_directories(../include)

add_executable(cnmatrix_test cn_matrixtest.c)
target_link_libraries(cnmatrix_test cnmatrix)

add_test(NAME cnmatrix_test COMMAND cnmatrix_test)
