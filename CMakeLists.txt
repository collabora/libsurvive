cmake_minimum_required(VERSION 3.12)

project(libsurvive
	LANGUAGES C CXX
	VERSION 0.3)
include(CheckIncludeFile)
include (CheckSymbolExists)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


SET(CORE_BUILD_DEFAULT OFF)
if(ANDROID)
  SET(CORE_BUILD_DEFAULT ON)
endif()

macro(VERBOSE_OPTION ARG DESC)
    option(${ARG} ${DESC} ${ARGN})
    message("-- Option: ${ARG}: ${ARGN}")
endmacro()

VERBOSE_OPTION(DO_CORE_BUILD "Only build essentials by default" ${CORE_BUILD_DEFAULT})
VERBOSE_OPTION(BUILD_STATIC "Build as static library" ${DO_CORE_BUILD})

VERBOSE_OPTION(USE_SINGLE_PRECISION "Use float instead of double" OFF)
VERBOSE_OPTION(ENABLE_WARNINGS_AS_ERRORS "Use to flag all warnings as errors" OFF)
SET(IS_WINDOWS OFF)
IF(WIN32)
    SET(IS_WINDOWS ON)
ENDIF()
VERBOSE_OPTION(USE_HIDAPI "Use HIDAPI instead of libusb" ${IS_WINDOWS})
VERBOSE_OPTION(USE_ASAN "Use address sanitizer" OFF)
VERBOSE_OPTION(USE_MSAN "Use memory sanitizer" OFF)
VERBOSE_OPTION(ENABLE_TESTS "Enable build / execution of tests" OFF)
VERBOSE_OPTION(USE_HEX_FLOAT_PRINTF "Use hex floats when recording" OFF)

VERBOSE_OPTION(USE_OPENCV "Use opencv proper for math operations" OFF)
VERBOSE_OPTION(USE_COLUMN_MAJOR_MATRICES "Use column major matrices for math operations" OFF)
VERBOSE_OPTION(USE_CPU_TUNE "Compile for your specific CPU" OFF)

find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache) # Less useful to do it for linking, see edit2
endif(CCACHE_FOUND)

if(USE_SINGLE_PRECISION)
  add_compile_definitions(CN_USE_FLOAT)
endif()

IF(WIN32)
    find_program(NUGET nuget)
    if(NOT NUGET)
      file(DOWNLOAD https://dist.nuget.org/win-x86-commandline/v5.6.0/nuget.exe "${CMAKE_BINARY_DIR}/nuget.exe"
        EXPECTED_MD5 3f8a53a524c54cf2e4efb7aea370a977 STATUS status LOG log)
      find_program(NUGET nuget HINTS "${CMAKE_BINARY_DIR}" )
    endif()
    message("Nuget is found at ${NUGET}")
    SET(CMAKE_VS_NUGET_PACKAGE_RESTORE ON)

    if (CMAKE_SIZEOF_VOID_P MATCHES 8)
      set(WIN_PLATFORM "x64")
      set(WIN_PLATFORMx "x64")
    else(CMAKE_SIZEOF_VOID_P MATCHES 8)
      set(WIN_PLATFORM "win32")
      set(WIN_PLATFORMx "x32")
    endif(CMAKE_SIZEOF_VOID_P MATCHES 8)

ENDIF()

VERBOSE_OPTION(USE_OPENBLAS "Use OpenBLAS" ${USE_OPENBLAS_DEFAULT})
VERBOSE_OPTION(BUILD_LH1_SUPPORT "Build LH1 support" ON)

if(BUILD_LH1_SUPPORT)
  add_definitions(-DBUILD_LH1_SUPPORT)
endif()

if(USE_HEX_FLOAT_PRINTF)
  add_definitions(-DSURVIVE_HEX_FLOATS)
endif()

set(CMAKE_REQUIRED_LIBRARIES z)

# Check Symbol exists doesn't like -Werror=pedantic
include(CheckCSourceCompiles)
check_c_source_compiles("
    #include <zlib.h>
    #include <stdarg.h>
    int main(void) {
      va_list args;
      gzvprintf(0, 0, args);
      return 0;
    }
  "
  HAVE_GZVPRINTF
)
if(NOT HAVE_GZVPRINTF)
  add_definitions(-DHAVE_NO_GZVPRINTF)
endif()

SET(SURVIVE_LIBRARY_TYPE SHARED)
if(BUILD_STATIC)
  SET(SURVIVE_LIBRARY_TYPE STATIC)
  add_definitions(-DSURVIVE_DISABLE_PLUGINS)
endif()

set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads)

check_include_file("gattlib.h" HAVE_GATTLIB_H)
VERBOSE_OPTION(BUILD_GATT_SUPPORT "Whether or not to include gatt support for the basestations -- requires gattlib (https://github.com/labapart/gattlib)" ${HAVE_GATTLIB_H})


check_include_file("zlib.h" HAVE_ZLIB_H)
IF(NOT HAVE_ZLIB_H OR ANDROID)
  add_definitions(-DNOZLIB)
ENDIF()

SET(NOT_CORE_BUILD ON)
if(DO_CORE_BUILD)
  SET(NOT_CORE_BUILD OFF)
endif()
VERBOSE_OPTION(BUILD_APPLICATIONS "Build the default libsurvive applications" ${NOT_CORE_BUILD})

#include_directories(${CMAKE_INSTALL_PREFIX}/include)
#link_directories(${CMAKE_INSTALL_PREFIX}/lib)

IF(UNIX)
  set(SHARED_FLAGS "-fPIC -Wall -Wno-unused-variable -Wno-switch -Wno-parentheses -Wno-missing-braces -Werror=return-type -fvisibility=hidden -Werror=vla -fno-math-errno -Werror=pointer-arith")

  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${SHARED_FLAGS} -std=gnu99 -Werror=incompatible-pointer-types -Werror=implicit-function-declaration -Werror=missing-field-initializers ")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${SHARED_FLAGS} -std=c++14")

  if(USE_CPU_TUNE)
      add_compile_options("-mtune=native")
  endif()

  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -rdynamic")

	if(ENABLE_WARNINGS_AS_ERRORS)
	  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Werror")
	endif()
	      	
	if(USE_ASAN)
	  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fsanitize=address -fsanitize=undefined")
	  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -fsanitize=undefined")
	endif()

	if(USE_MSAN)
	  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fsanitize=memory -fsanitize-memory-track-origins -fsanitize=undefined")
	  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=memory  -fsanitize-memory-track-origins -fsanitize=undefined")
	endif()	    
endif()

include_directories(redist include/libsurvive include)

if(USE_SINGLE_PRECISION)
    add_definitions(-DUSE_FLOAT)
endif()

IF(ENABLE_TESTS)
  enable_testing()
ENDIF()

 find_library(OPENVR_LIBRARIES
              NAMES
              openvr_api
              openvr_api64
              PATH_SUFFIXES
              osx32
              linux64
              PATHS
              "C:/Program Files (x86)/OpenVRSDK/lib"
)

find_path(OPENVR_INCLUDE_PATH
          NAMES openvr.h openvr_driver.h
          PATH_SUFFIXES
          openvr
          "C:/Program Files (x86)/OpenVRSDK/include"
)

SET(CMAKE_SKIP_BUILD_RPATH  FALSE)
SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

add_subdirectory(libs/cnmatrix)
add_subdirectory(libs/cnkalman)

add_subdirectory(redist)
add_subdirectory(src)
add_subdirectory(tools)

SET(SURVIVE_EXECUTABLES survive-cli api_example sensors-readout survive-solver survive-buttons)
foreach(executable ${SURVIVE_EXECUTABLES})
  VERBOSE_OPTION(ENABLE_${executable} "Build ${executable}" ${BUILD_APPLICATIONS})

  if(${executable} STREQUAL "survive-cli" AND WIN32)
	set(ENABLE_survive-cli ON)
  endif()

  if(ENABLE_${executable})
    add_executable(${executable} ${executable}.c )
    target_link_libraries(${executable} survive ${${executable}_ADDITIONAL_LIBS} ${CMAKE_THREAD_LIBS_INIT})
    set_target_properties(${executable} PROPERTIES FOLDER "apps")
    foreach(plugin ${SURVIVE_BUILT_PLUGINS})
      add_dependencies(${executable} ${plugin})
    endforeach()
    install(TARGETS ${executable} DESTINATION bin/${CMAKE_GENERATOR_PLATFORM})
  endif()
endforeach()

IF(WIN32)
  # This property needs to be set on a target at the root; doesn't matter which
  set_target_properties( survive-cli PROPERTIES VS_USER_PROPS "$(SolutionDir)\\packages\\OpenBLAS.0.2.14.1\\build\\native\\openblas.targets" )
endif()

SET(SURVIVE_WEBSOCKETD ${CMAKE_CURRENT_SOURCE_DIR}/useful_files/survive-websocketd)
if(WIN32)
	SET(SURVIVE_WEBSOCKETD ${CMAKE_CURRENT_SOURCE_DIR}/useful_files/survive-websocketd.ps1)
endif()

IF(TARGET survive-cli)
  add_custom_command(TARGET survive-cli POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy ${SURVIVE_WEBSOCKETD} $<TARGET_FILE_DIR:survive-cli>
    )
  install(PROGRAMS ${SURVIVE_WEBSOCKETD} DESTINATION bin/${CMAKE_GENERATOR_PLATFORM})
endif()

find_program(DOTNET dotnet)
if(DOTNET)
  SET(DOTNET_RUNTIME_FLAGS "")
  if(WIN32)
	if(CMAKE_SIZEOF_VOID_P EQUAL 8)
		#SET(DOTNET_RUNTIME_FLAGS "--runtime;win-x64")

	elseif(CMAKE_SIZEOF_VOID_P EQUAL 4)
		#SET(DOTNET_RUNTIME_FLAGS "--runtime;win-x86")
	endif()
  endif()
  
	message("${DOTNET} build -c Release ${DOTNET_RUNTIME_FLAGS}")
  execute_process(
    COMMAND ${DOTNET} build -c Release ${DOTNET_RUNTIME_FLAGS}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bindings/cs/)

  file(GLOB_RECURSE CS_BINDINGS
    "${CMAKE_CURRENT_SOURCE_DIR}/bindings/cs/libsurvive.net/bin/Release/*.dll")

  install(FILES ${CS_BINDINGS} DESTINATION "bin")

  if(WIN32)
    file(GLOB_RECURSE CS_DEMO
      "${CMAKE_CURRENT_SOURCE_DIR}/bindings/cs/Demo/bin/Release/Demo.*")

    install(FILES ${CS_DEMO} DESTINATION "bin")
  endif()
endif()

if(EXISTS ${CMAKE_INSTALL_PREFIX}/etc/bash_completion.d)
  install(PROGRAMS ${CMAKE_CURRENT_SOURCE_DIR}/survive_autocomplete.sh DESTINATION "${CMAKE_INSTALL_PREFIX}/etc/bash_completion.d")
endif()

install(DIRECTORY include/libsurvive DESTINATION include)

file(GLOB REDIST_HEADERS
  "redist/*.h"
)
install(FILES ${REDIST_HEADERS} DESTINATION include/libsurvive/redist)

if(NOT WIN32 AND PYTHON_GENERATED_DIR)

  find_program(CTYPESGEN ctypesgen)

  if(CTYPESGEN)
    message("-- Building python bindings file to ${PYTHON_GENERATED_DIR}")
    message("-- ctypesgen found at ${CTYPESGEN}") 
    get_property(include_directories DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
    set(INCLUDE_FLAGS "")
    foreach(include_directory ${include_directories})
      list(APPEND INCLUDE_FLAGS "-I${include_directory}")
    endforeach()

    add_custom_target(pysurvive ALL COMMAND ${Python3_EXECUTABLE} ${CTYPESGEN} ${CMAKE_SOURCE_DIR}/include/libsurvive/*.h ${INCLUDE_FLAGS} --no-macros -L$<TARGET_FILE_DIR:survive> -lsurvive
            --strip-prefix=survive_ -P Survive -o ${PYTHON_GENERATED_DIR}pysurvive_generated.py)
  endif()

endif()

include(GNUInstallDirs)
configure_file(survive.pc.in survive.pc @ONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/survive.pc" DESTINATION "${CMAKE_INSTALL_LIBDIR}/pkgconfig")
