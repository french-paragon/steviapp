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

find_package(QT NAMES Qt6 Qt5 COMPONENTS Widgets Core REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets Core REQUIRED)
find_package(exiv2 REQUIRED CONFIG NAMES exiv2)

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
    control/landmarkbaseactionmanager.cpp
    control/landmarkbaseactionmanager.h
    control/solversactions.cpp
    control/solversactions.h)

add_executable(steviapp
    ${PROJ_SRC}
)

target_link_libraries(steviapp PRIVATE Qt${QT_VERSION_MAJOR}::Widgets)
target_link_libraries(steviapp PRIVATE exiv2)

target_link_libraries(steviapp PRIVATE ${DATA_LIB} ${GEOMETRY_LIB} ${GUI_LIB} ${PROCESSING_LIB} ${SOLVING_LIB})
