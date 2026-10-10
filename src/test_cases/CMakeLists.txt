SET(SURVIVE_TESTS
        reproject
        check_generated barycentric_svd optimizer
        rotate_angvel export_config)

set(barycentric_svd_ADDITIONAL_SRCS ../barycentric_svd/barycentric_svd.c)

# Regression test for two bugs in handle_transfer()'s USB transfer retry
# logic; see the comment at the top of driver_vive_timeout_stall_test.c.
# Compiles driver_vive.c directly (its functions under test are `static`)
# against a fake libusb (mock_libusb.c) instead of the real one, so it
# must NOT link driver_vive or ${LIBUSB_LIBRARY} -- survive's core target
# doesn't pull libusb in on its own (only the driver_vive plugin does), so
# plain `survive` is safe.
IF(NOT WIN32 AND NOT USE_HIDAPI)
    LIST(APPEND SURVIVE_TESTS driver_vive_timeout_stall_test)
    set(driver_vive_timeout_stall_test_ADDITIONAL_SRCS mock_libusb.c)
endif()

IF(NOT WIN32)
    LIST(APPEND SURVIVE_TESTS watchman)
    set(watchman_ADDITIONAL_LIBS driver_vive)
endif()
SET(SURVIVE_TESTS_EXE)
foreach(test ${SURVIVE_TESTS})
    list(APPEND SURVIVE_TESTS_EXE test-${test})
    add_executable(test-${test} ${test}.c main.c ${${test}_ADDITIONAL_SRCS})
    target_link_libraries(test-${test} survive ${${test}_ADDITIONAL_LIBS})
    if(${test} STREQUAL "driver_vive_timeout_stall_test")
      # -g -O0: keep real stack frames for debugging failures this test
      # finds; optimized/inlined frames obscure them.
      target_compile_options(test-${test} PRIVATE -g -O0)
    endif()
    add_test(NAME ${test} COMMAND test-${test})
    add_dependencies(test-${test} survive_plugins)
    set_target_properties(test-${test} PROPERTIES FOLDER "tests")
endforeach()

add_definitions(-DDEBUG_WATCHMAN)

add_executable(test_replays test_replays.c)
set_target_properties(test_replays PROPERTIES FOLDER "tests")
add_dependencies(test_replays ${SURVIVE_BUILT_PLUGINS})
target_link_libraries(test_replays survive)

if(NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/libsurvive-extras-data)
    execute_process(COMMAND git clone https://github.com/jdavidberger/libsurvive-extras-data.git ${CMAKE_CURRENT_BINARY_DIR}/libsurvive-extras-data)
endif()

# These tests were broken by a recent change and work needs to be done to fix them. Specifically this is targeted
# for a change that let LH be repositioned when it seemed the measurements were off but too many users reported this
# led perfectly fine LH positions being discarded.
SET(TEST_EXCLUSION_LIST
        ${CMAKE_CURRENT_BINARY_DIR}/libsurvive-extras-data/tests/tracker-lh-move.pcap.gz
        ${CMAKE_CURRENT_BINARY_DIR}/libsurvive-extras-data/tests/flip_out_lh_solve.rec.gz
        ${CMAKE_CURRENT_BINARY_DIR}/libsurvive-extras-data/tests/gss_trial1_trim.rec.gz
        ${CMAKE_CURRENT_BINARY_DIR}/libsurvive-extras-data/tests/openvr-shake-comparison-test.rec.gz

# Ignore this test for now; we don't have good ootx for it
    ${CMAKE_CURRENT_BINARY_DIR}/libsurvive-extras-data/tests/index-yank-up-down.rec.gz
  )

IF(NOT WIN32)
    file(GLOB REC_FILES ${CMAKE_CURRENT_BINARY_DIR}/libsurvive-extras-data/tests/*.rec.gz)
    list(REMOVE_ITEM REC_FILES ${TEST_EXCLUSION_LIST})

    foreach(REC_FILE ${REC_FILES})
        get_filename_component(REC_FILE_NAME ${REC_FILE} NAME)
        add_test(NAME ${REC_FILE_NAME} COMMAND $<TARGET_FILE:test_replays> ${REC_FILE})
    endforeach()
ENDIF()

if(PCAP_LIBRARY)
    file(GLOB REC_FILES ${CMAKE_CURRENT_BINARY_DIR}/libsurvive-extras-data/tests/*.pcap.gz)
    list(REMOVE_ITEM REC_FILES ${TEST_EXCLUSION_LIST})

    foreach(REC_FILE ${REC_FILES})
        get_filename_component(REC_FILE_NAME ${REC_FILE} NAME)
        add_test(NAME ${REC_FILE_NAME}_pcap COMMAND $<TARGET_FILE:test_replays> ${REC_FILE})
    endforeach()

    # This calibrates a wireless tracker with 4 lh. Originally OOTX didn't work.
    add_test(NAME tracker-cal-attempt-pcap-no-ootx COMMAND $<TARGET_FILE:test_replays> ${CMAKE_CURRENT_BINARY_DIR}/libsurvive-extras-data/tests/tracker-cal-attempt.pcap.gz --force-ootx)
endif()

include(ExternalProject)

ExternalProject_Add(libsurvive-extras-data
        GIT_REPOSITORY https://github.com/jdavidberger/libsurvive-extras-data.git
        BUILD_COMMAND ""
        INSTALL_COMMAND ""
        CONFIGURE_COMMAND ""
        UPDATE_DISCONNECTED ON
        SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/libsurvive-extras-data
        )

add_custom_target(run_all_tests COMMAND ${CMAKE_CTEST_COMMAND} -j 30 --output-on-failure DEPENDS test_replays libsurvive-extras-data survive_plugins ${SURVIVE_TESTS_EXE})
