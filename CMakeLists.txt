cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0095 NEW)

option(BUILD_TESTS "Build the test suite" ON)
option(ADVANCED_LOGGING "Log a lot of information, usefull if you want to investigate bug reports from users." ON)

include(CheckCXXCompilerFlag)
project(steviapp LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DMULTIDIMARRAYS_STRONG_BOUND_CHECK")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -DMULTIDIMARRAYS_STRONG_BOUND_CHECK")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -fopenmp -march=native -ftree-vectorize")

#options
option(ADD_DEVEL_TOOLS "enable devellopement tools in the build" OFF)
option(BUILD_SHARED_LIBS "Build using shared libraries" ON)
set(STEVIAPP_DEVEL_TOOLS ${ADD_DEVEL_TOOLS})

#some global variables

include(GNUInstallDirs)

include(FetchContent)

set(CMAKE_INSTALL_BINDIR ${CMAKE_INSTALL_BINDIR})
set(CMAKE_INSTALL_CMAKEDIR ${CMAKE_INSTALL_LIBDIR})
set(CMAKE_INSTALL_LIBDIR ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME})
set(CMAKE_INSTALL_INCLUDEDIR ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME})

set(CONFIGURED_INCLUDES_DIR ${CMAKE_BINARY_DIR}/configured_includes)
include_directories(${CONFIGURED_INCLUDES_DIR})

message("Lib install dir: ${CMAKE_INSTALL_LIBDIR}")

set(STEVIAPP_SDK_NAME "SteviappSdk")
set(STEVIAPP_SDK_EXPORTNAME "SteviappSdkTargets")

set(CMAKE_INSTALL_RPATH "$ORIGIN/../${CMAKE_INSTALL_LIBDIR}:$ORIGIN/../../${CMAKE_INSTALL_LIBDIR}")

#SIMD for x86
check_cxx_compiler_flag("-mavx" AVXSUPPORTED)
check_cxx_compiler_flag("-mavx2" AVX2SUPPORTED)
check_cxx_compiler_flag("-mfma" FMASUPPORTED)

if (AVXSUPPORTED)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -mavx")
endif()

if (AVX2SUPPORTED)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -mavx2")
endif()

if (FMASUPPORTED)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -mfma")
endif()

#backtrace limit
check_cxx_compiler_flag("-ftemplate-backtrace-limit=100" TMPL_BACKTRACE_LIMIT)

if (TMPL_BACKTRACE_LIMIT)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ftemplate-backtrace-limit=100")
endif()

# QtCreator supports the following variables for Android, which are identical to qmake Android variables.
# Check http://doc.qt.io/qt-5/deployment-android.html for more information.
# They need to be set before the find_package(Qt5 ...) call.

#if(ANDROID)
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
#    if (ANDROID_ABI STREQUAL "armeabi-v7a")
#        set(ANDROID_EXTRA_LIBS
#            ${CMAKE_CURRENT_SOURCE_DIR}/path/to/libcrypto.so
#            ${CMAKE_CURRENT_SOURCE_DIR}/path/to/libssl.so)
#    endif()
#endif()

#MultidimArrays

FetchContent_Declare(
  MultidimArrays
  GIT_REPOSITORY https://github.com/french-paragon/MultidimArrays.git
  GIT_TAG 78d674a3a3713feee4ced37a95fef3b53cc2832b
  FIND_PACKAGE_ARGS
)
FetchContent_MakeAvailable(MultidimArrays)

find_package(QT NAMES Qt5 COMPONENTS Widgets Core Test PrintSupport REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets Core Test PrintSupport REQUIRED)
find_package(exiv2 REQUIRED CONFIG NAMES exiv2)

find_package(Eigen3 3.3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

FetchContent_Declare(
  StereoVision
  GIT_REPOSITORY https://github.com/french-paragon/LibStevi.git
  GIT_TAG 2bcd37d292af6404a2aa6c9730c9a73577d5262b
  FIND_PACKAGE_ARGS
)
set(buildGui ON) #ensuire gui is built for stereo vision
FetchContent_MakeAvailable(StereoVision)

if (ADVANCED_LOGGING)
        FetchContent_Declare(
          cpptrace
          GIT_REPOSITORY https://github.com/jeremy-rifkin/cpptrace.git
          GIT_TAG        v0.8.3
    )
    FetchContent_MakeAvailable(cpptrace)
endif(ADVANCED_LOGGING)

find_package(PkgConfig REQUIRED)
pkg_check_modules(PROJ REQUIRED IMPORTED_TARGET proj)

add_library(PROJ::proj ALIAS PkgConfig::PROJ)
#TODO: check when the ubuntu repos update to the PROJ 9 release, which should include a cmake package

find_package(glog REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

#python module
find_package(pybind11 REQUIRED)

#configure submodules
add_subdirectory(modules)
include_directories(${MODULES_INCLUDE_DIRECTORIES} libs)
link_directories(${MODULES_LINK_DIRECTORIES})

add_subdirectory(libs)

set(PROJ_SRC  main.cpp
    )

add_executable(steviapp
    ${PROJ_SRC}
)

target_link_libraries(steviapp PRIVATE Qt${QT_VERSION_MAJOR}::Core Qt${QT_VERSION_MAJOR}::Widgets pybind11::embed glog::glog)

target_link_libraries(steviapp PRIVATE StereoVision::stevi MultidimArrays::MultidimArrays ${DATA_LIB} ${VISION_LIB} ${GUI_LIB} ${PROCESSING_LIB} ${SOLVING_LIB} ${CONTROL_LIB} Threads::Threads)

if (ADVANCED_LOGGING)
        target_compile_definitions(steviapp PRIVATE ADVANCED_LOGGING)
        target_link_libraries(steviapp PRIVATE cpptrace::cpptrace)
endif()

pybind11_add_module(pysteviapp SHARED python_api.cpp)

set_target_properties(pysteviapp PROPERTIES DEBUG_POSTFIX "")

target_link_libraries(pysteviapp PRIVATE Qt${QT_VERSION_MAJOR}::Core Qt${QT_VERSION_MAJOR}::Widgets)
target_link_libraries(pysteviapp PRIVATE StereoVision::stevi MultidimArrays::MultidimArrays ${DATA_LIB} ${VISION_LIB} ${GUI_LIB} ${PROCESSING_LIB} ${SOLVING_LIB} ${CONTROL_LIB} Threads::Threads)


if (BUILD_TESTS)
        add_subdirectory(tests)
endif (BUILD_TESTS)

#installing
install(
  EXPORT ${STEVIAPP_SDK_EXPORTNAME}
  DESTINATION ${CMAKE_INSTALL_CMAKEDIR}/cmake/${STEVIAPP_SDK_NAME}
  NAMESPACE ${STEVIAPP_SDK_NAME}::
  FILE SteviappSDKTargets.cmake # ensure the name is different than the SteviappSDKConfig.cmake file
  )

include(CMakePackageConfigHelpers)

configure_package_config_file(
  "cmake/Config.cmake.in"
  "${STEVIAPP_SDK_NAME}Config.cmake"
  INSTALL_DESTINATION ${CMAKE_INSTALL_CMAKEDIR}/cmake/${STEVIAPP_SDK_NAME}
  PATH_VARS
    CMAKE_INSTALL_LIBDIR
  )

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/${STEVIAPP_SDK_NAME}ConfigVersion.cmake"
  VERSION "0.0"
  COMPATIBILITY AnyNewerVersion
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${STEVIAPP_SDK_NAME}Config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/${STEVIAPP_SDK_NAME}ConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_CMAKEDIR}/cmake/${STEVIAPP_SDK_NAME} )

install (
    TARGETS steviapp
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
