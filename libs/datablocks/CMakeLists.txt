define_stereovisionapp_lib(DATA Data)

set(LIB_SRC floatparameter.cpp
            floatparameter.h
            dataexception.cpp
            dataexception.h
            project.cpp
            project.h
            landmark.cpp
            landmark.h
            image.cpp
            image.h
            camera.cpp
            camera.h
            itemdatamodel.cpp
            itemdatamodel.h
            stereorig.cpp
            stereorig.h
            angleconstrain.cpp
            angleconstrain.h
            distanceconstrain.cpp
            distanceconstrain.h
            landmarkscollection.cpp
            landmarkscollection.h
            io/collada_export_filter.cpp
            io/collada_export_filter.h
)

add_library(
    ${DATA_LIB}
    ${LIB_SRC}
)


target_link_libraries(${DATA_LIB} PRIVATE Qt${QT_VERSION_MAJOR}::Core Qt${QT_VERSION_MAJOR}::Widgets)
