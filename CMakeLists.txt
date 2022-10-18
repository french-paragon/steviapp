cmake_minimum_required(VERSION 3.5)

include(CheckCXXCompilerFlag)
project(steviapp LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -fopenmp -march=native -ftree-vectorize")

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

find_package(QT NAMES Qt6 Qt5 COMPONENTS Widgets Core Test REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets Core Test REQUIRED)
find_package(exiv2 REQUIRED CONFIG NAMES exiv2)

find_package(Eigen3 3.3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

#configure submodules
add_subdirectory(modules)
include_directories(${MODULES_INCLUDE_DIRECTORIES} libs)
message(${MODULES_LINK_DIRECTORIES})
link_directories(${MODULES_LINK_DIRECTORIES})

add_subdirectory(libs)

set(PROJ_SRC  main.cpp
    mainwindow.cpp
    mainwindow.h
    mainwindow.ui
    control/actionmanager.cpp
    control/actionmanager.h
    control/imagebaseactions.cpp
    control/imagebaseactions.h
    control/imagebaseactionmanager.cpp
    control/imagebaseactionmanager.h
    control/camerabaseactionmanager.cpp
    control/camerabaseactionmanager.h
	control/landmarkbasedactions.cpp
	control/landmarkbasedactions.h
    control/landmarkbaseactionmanager.cpp
    control/landmarkbaseactionmanager.h
    control/stereorigactionmanager.cpp
    control/stereorigactionmanager.h
    control/angleconstrainactionmanager.cpp
    control/angleconstrainactionmanager.h
    control/distanceconstrainactionmanager.cpp
    control/distanceconstrainactionmanager.h
    control/solversactions.cpp
    control/solversactions.h
    control/exportactions.cpp
    control/exportactions.h
    control/cameracalibrationactionmanager.cpp
    control/cameracalibrationactionmanager.h
    control/cameracalibrationactions.cpp
    control/cameracalibrationactions.h
    )

add_executable(steviapp
    ${PROJ_SRC}
)

target_link_libraries(steviapp PRIVATE Qt${QT_VERSION_MAJOR}::Widgets)
target_link_libraries(steviapp PRIVATE exiv2)

target_link_libraries(steviapp PRIVATE ${STEREOVISION_LIB} ${DATA_LIB} ${VISION_LIB} ${GUI_LIB} ${PROCESSING_LIB} ${SOLVING_LIB} Threads::Threads)
