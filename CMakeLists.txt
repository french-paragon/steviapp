cmake_minimum_required(VERSION 3.5)

project(steviapp LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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

find_package(QT NAMES Qt6 Qt5 COMPONENTS Widgets REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets REQUIRED)

#configure submodules
add_subdirectory(modules)
include_directories(${MODULES_INCLUDE_DIRECTORIES})
message(${MODULES_LINK_DIRECTORIES})
link_directories(${MODULES_LINK_DIRECTORIES})

set(PROJ_SRC  main.cpp
    mainwindow.cpp
    mainwindow.h
    mainwindow.ui
    gui/imagewidget.cpp
    gui/imagewidget.h
    gui/editor.cpp
    gui/editor.h
    gui/imageeditor.cpp
    gui/imageeditor.h
    gui/imageeditor.ui
    datablocks/project.cpp
    datablocks/project.h
    datablocks/floatparameter.cpp
    datablocks/floatparameter.h
    datablocks/itemdatamodel.cpp
    datablocks/itemdatamodel.h
    datablocks/landmark.cpp
    datablocks/landmark.h
    datablocks/image.cpp
    datablocks/image.h
    datablocks/camera.cpp
    datablocks/camera.h
    sparsesolver/sbagraphreductor.cpp
    sparsesolver/sbagraphreductor.h
    sparsesolver/graphsbasolver.h
    sparsesolver/graphsbasolver.cpp
    sparsesolver/vertices/camerapose.cpp
    sparsesolver/vertices/camerapose.h
    sparsesolver/vertices/vertexcamerapose.cpp
    sparsesolver/vertices/vertexcamerapose.h
    sparsesolver/vertices/camparam.cpp
    sparsesolver/vertices/camparam.h
    sparsesolver/vertices/vertexcameraparam.cpp
    sparsesolver/vertices/vertexcameraparam.h
    sparsesolver/vertices/vertexcameraradialdistortion.cpp
    sparsesolver/vertices/vertexcameraradialdistortion.h
    sparsesolver/vertices/vertexcameratangentialdistortion.cpp
    sparsesolver/vertices/vertexcameratangentialdistortion.h
    sparsesolver/vertices/vertexcameraskewdistortion.cpp
    sparsesolver/vertices/vertexcameraskewdistortion.h
    sparsesolver/edges/edgeparametrizedxyz2uv.cpp
    sparsesolver/edges/edgeparametrizedxyz2uv.h
    sparsesolver/edges/edgese3fullprior.cpp
    sparsesolver/edges/edgese3fullprior.h
    sparsesolver/edges/edgese3rpyprior.cpp
    sparsesolver/edges/edgese3rpyprior.h
    sparsesolver/edges/edgese3xyzprior.cpp
    sparsesolver/edges/edgese3xyzprior.h
    sparsesolver/edges/edgexyzprior.cpp
    sparsesolver/edges/edgexyzprior.h)

add_executable(steviapp
    ${PROJ_SRC}
)

target_link_libraries(steviapp PRIVATE Qt${QT_VERSION_MAJOR}::Widgets)
target_link_libraries(steviapp PRIVATE core types_sba solver_csparse)
