SET(CN_MATRIX_SRCS ../include/cnmatrix/cn_matrix.h cn_matrix.c)

IF(USE_EIGEN)
  SET(CN_MATRIX_SRCS ${CN_MATRIX_SRCS} eigen/core.cpp eigen/gemm.cpp eigen/svd.cpp eigen/internal.h)
ELSE()
  SET(CN_MATRIX_SRCS ${CN_MATRIX_SRCS} cn_matrix.blas.c)
ENDIF()

IF(WIN32)
  set(packages_config "<packages><package id=\"lapacke\" version=\"0.1.0\" targetFramework=\"native\" /><package id=\"OpenBLAS\" version=\"0.2.14.1\" targetFramework=\"native\" /></packages>")
  file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/packages.config ${packages_config})
  ADD_DEFINITIONS(-DHAVE_LAPACK_CONFIG_H -DLAPACK_COMPLEX_STRUCTURE)
  set(CN_MATRIX_SRCS ${CN_MATRIX_SRCS} ${CMAKE_CURRENT_BINARY_DIR}/packages.config)
endif()

macro(find_library_path var pkg)
  find_library(${var}_RAWFIND ${pkg} ${ARGN})
  if(${var}_RAWFIND)
    set(${var} ${${var}_RAWFIND})
    message("Found ${pkg} at ${${var}}")
  else()
    message("Could not find ${pkg}")
    set(${var} ${pkg})
  endif()
endmacro()
find_library_path(CBLAS_LOCATION cblas HINTS /usr/local/opt/openblas/lib)
find_library_path(OPENBLAS_LOCATION openblas HINTS /usr/local/opt/openblas/lib)
find_library_path(LAPACKE_LOCATION lapacke HINTS /usr/local/opt/lapack/lib)

if(USE_OPENBLAS OR WIN32)
  set(BLAS_BACKEND "openblas")
  include_directories(openblas)
else()
  set(BLAS_BACKEND "${CBLAS_LOCATION}")  
  find_file(CBLAS_FULL_PATH cblas.h HINT "/usr/local/opt/openblas/include")
  message("Searching for 'cblas.h'; found at ${CBLAS_FULL_PATH}")
  IF(CBLAS_FULL_PATH)
    get_filename_component(CBLAS_PATH ${CBLAS_FULL_PATH} DIRECTORY)
    include_directories( ${CBLAS_PATH})
  ENDIF()
endif()


check_include_file(lapacke/lapacke.h LAPACKE_FILE)
if(LAPACKE_FILE)
  add_definitions(-DLAPACKE_FOLDER)
endif()

if(USE_EIGEN)
    message("Using eigen backend")
else()
    message("Using blas backend ${BLAS_BACKEND}")
endif()

add_library(cnmatrix STATIC ${CN_MATRIX_SRCS})
target_include_directories(cnmatrix PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include/>
  $<INSTALL_INTERFACE:include/>)

if(USE_SINGLE_PRECISION)
  target_compile_definitions(cnmatrix PUBLIC CN_USE_FLOAT)
endif()

set_target_properties(cnmatrix PROPERTIES FOLDER "libraries")

IF(USE_EIGEN)
  add_definitions     ( ${EIGEN3_DEFINITIONS} )
  include_directories ( ${EIGEN3_INCLUDE_DIR} )

  if(WIN32)
    target_compile_options(cnmatrix PRIVATE -bigobj)
  else()
    target_compile_options(cnmatrix PRIVATE -fno-exceptions -fPIC)
  endif()
ELSE()
  IF(UNIX)
    target_link_libraries(cnmatrix ${BLAS_BACKEND} ${LAPACKE_LOCATION} m)
  elseif(WIN32)
    include_directories(${CMAKE_BINARY_DIR}/packages/OpenBLAS.0.2.14.1/lib/native/include/)
    target_link_libraries(cnmatrix
      ${CMAKE_BINARY_DIR}/packages/OpenBLAS.0.2.14.1/lib/native/lib/${WIN_PLATFORM}/libopenblas.dll.a
      )
  ENDIF()
ENDIF()

install(TARGETS cnmatrix DESTINATION lib)
