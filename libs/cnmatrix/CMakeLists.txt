cmake_minimum_required(VERSION 3.12)

project(cnmatrix LANGUAGES C CXX)
include(CTest)

option(ENABLE_TESTS "Enable tests" OFF)
if (${ENABLE_TESTS})
    enable_testing()
endif()
  
include(CheckIncludeFile)
include (CheckSymbolExists)


macro(CNMATRIX_OPTION ARG DESC)
  option(${ARG} ${DESC} ${ARGN})
  SET(${ARG} ${ARGN})
  message("-- CNMatrix option: ${ARG}: ${ARGN}")
endmacro()

CNMATRIX_OPTION(DOWNLOAD_EIGEN "Download eigen if it isn't installed on the system" ON)
CNMATRIX_OPTION(USE_EIGEN "Use eigen for math operations" ${DOWNLOAD_EIGEN})
CNMATRIX_OPTION(USE_COLUMN_MAJOR_MATRICES "Use column major matrices for math operations" OFF)
CNMATRIX_OPTION(USE_SINGLE_PRECISION "Use floats instead of doubles" OFF)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package (Eigen3 3.3 QUIET)
if(NOT EIGEN3_FOUND)
  if(DOWNLOAD_EIGEN)
    find_program(GIT git)
    if(NOT EXISTS "${CMAKE_CURRENT_BINARY_DIR}/libeigen-src/Eigen")
      execute_process(COMMAND ${GIT} clone -b 3.4 https://gitlab.com/libeigen/eigen.git "${CMAKE_CURRENT_BINARY_DIR}/libeigen-src")
    endif()
    set(EIGEN3_INCLUDE_DIR "${CMAKE_CURRENT_BINARY_DIR}/libeigen-src/")
    set(USE_EIGEN ON)
  else()
    message(WARNING "Could not find eigen; falling back to other backend")
    set(USE_EIGEN OFF)
  endif()
else()
  MESSAGE("Found system eigen")
endif()

IF(USE_EIGEN)
    add_compile_definitions(USE_EIGEN)

    if(USE_COLUMN_MAJOR_MATRICES)
        add_compile_definitions(CN_MATRIX_IS_COL_MAJOR)
    endif()
ENDIF()

find_library(CBLAS_LIB NAMES cblas)
set(USE_OPENBLAS_DEFAULT OFF)
if(NOT CBLAS_LIB)
  if(NOT USE_EIGEN)
    set(USE_OPENBLAS_DEFAULT ON)
  endif()
endif()

CNMATRIX_OPTION(USE_OPENBLAS "Use OpenBLAS" ${USE_OPENBLAS_DEFAULT})

install(DIRECTORY include/cnmatrix DESTINATION include)
include(GNUInstallDirs)
configure_file(cnmatrix.pc.in cnmatrix.pc @ONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/cnmatrix.pc" DESTINATION "${CMAKE_INSTALL_LIBDIR}/pkgconfig")

IF(UNIX)
  set(SHARED_FLAGS "-fPIC -Wall -Wno-unused-variable -Wno-switch -Wno-parentheses -Wno-missing-braces -Werror=return-type -fvisibility=hidden -Werror=vla -fno-math-errno -Werror=missing-field-initializers -Werror=pointer-arith")

  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${SHARED_FLAGS} -std=gnu99 -Werror=incompatible-pointer-types -Werror=implicit-function-declaration ")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${SHARED_FLAGS} -std=c++14")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -rdynamic")

  if(ENABLE_WARNINGS_AS_ERRORS)
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Werror")
  endif()

ELSEIF(WIN32)
  add_compile_options(/wd4244 /wd4996 /wd4018 /wd4101 /wd4477 /wd4068 /wd4217)
  set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
  set (CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
endif()


add_subdirectory(src)

if(ENABLE_TESTS)
  add_subdirectory(tests)
endif()

