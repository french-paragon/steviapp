define_stereovisionapp_lib(DATA Data)

set(LIB_SRC floatparameter.cpp
            dataexception.cpp
            project.cpp
            trajectory.cpp
            rigidbody.cpp
            point3d.cpp
            landmark.cpp
            image.cpp
            camera.cpp
            itemdatamodel.cpp
            genericcorrespondences.cpp
            correspondencesset.cpp
            stereorig.cpp
            angleconstrain.cpp
            distanceconstrain.cpp
            landmarkscollection.cpp
            cameracalibration.cpp
            localcoordinatesystem.cpp
            exportablestereosequence.cpp
            fixedstereopluscolorsequence.cpp
            fixedcolorstereosequence.cpp
            datatable.cpp
            cameras/pushbroompinholecamera.cpp
            io/collada_export_filter.cpp
            io/collada_export_filter.h
            io/jsonconversionsutils.h
)

set(LIB_HEADERS
    floatparameter.h
    dataexception.h
    georeferenceddatablockinterface.h
    project.h
    trajectory.h
    rigidbody.h
    point3d.h
    landmark.h
    image.h
    camera.h
    itemdatamodel.h
    genericcorrespondences.h
    correspondencesset.h
    stereorig.h
    angleconstrain.h
    distanceconstrain.h
    landmarkscollection.h
    cameracalibration.h
    localcoordinatesystem.h
    exportablestereosequence.h
    fixedstereopluscolorsequence.h
    fixedcolorstereosequence.h
    datatable.h
)

set(LIB_CAMERAS_HEADERS
    cameras/pushbroompinholecamera.h
)

add_library(
    ${DATA_LIB}
    ${LIB_SRC}
    ${LIB_HEADERS}
    ${LIB_CAMERAS_HEADERS}
)

target_link_libraries(${DATA_LIB} PRIVATE Qt${QT_VERSION_MAJOR}::Core Qt${QT_VERSION_MAJOR}::Widgets PROJ::proj)

install_stereovisionapp_sdklib(${DATA_LIB})

get_filename_component(LibDirName ${CMAKE_CURRENT_SOURCE_DIR} NAME)

install(FILES ${LIB_HEADERS}
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${LibDirName}
  )

install(FILES ${LIB_CAMERAS_HEADERS}
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${LibDirName}/cameras
  )
