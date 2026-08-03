include_directories(../include)

include_directories(.)

cnkalman_generate_code(./models/BikeLandmarks.py)
cnkalman_generate_code(./models/BearingsOnlyTracking.py)
cnkalman_generate_code(./models/EggLandscape.py)
cnkalman_generate_code(./models/BearingAccel.py)
add_executable(TestKalmanModels ModelRunner.cc TestKalmanModels.cc models/LinearPoseVel.h ./models/BikeLandmarks.gen.h
        ./models/EggLandscape.gen.h ./models/BearingsOnlyTracking.gen.h ./models/BearingAccel.gen.h)
target_link_libraries(TestKalmanModels cnkalman)

add_executable(kalman_example kalman_example.c)
target_link_libraries(kalman_example cnkalman)

add_test(NAME runbenchmarks
        COMMAND ${CMAKE_COMMAND}
        -Dcnkalman_root_source_dir=${cnkalman_root_source_dir}
        -DTEST_KALMAN_MODELS=$<TARGET_FILE:TestKalmanModels>
        -DBENCHMARK_SCRIPT=${CMAKE_CURRENT_SOURCE_DIR}/benchmark.py
        -P ${CMAKE_CURRENT_SOURCE_DIR}/runbenchmarks.cmake)

add_test(NAME TestKalmanModels COMMAND TestKalmanModels)


find_package(GTest)
if(GTest_FOUND)
    include(GoogleTest)

    add_executable(kalman_toy kalman_toy.cc)
    target_link_libraries(kalman_toy cnkalman GTest::gtest GTest::gtest_main)
    gtest_discover_tests(kalman_toy)

    add_executable(cnkalman-tests multi_model_test.cc feature_tests.cc)
    target_link_libraries(cnkalman-tests cnkalman GTest::gtest GTest::gtest_main)
    gtest_discover_tests(cnkalman-tests)
endif()
