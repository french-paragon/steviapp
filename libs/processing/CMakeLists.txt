define_stereovisionapp_lib(PROCESSING Processing)

set(LIB_SRC steppedprocess.cpp
            steppedprocess.h
)

add_library(
    ${PROCESSING_LIB}
    ${LIB_SRC}
)


target_link_libraries(${PROCESSING_LIB} PRIVATE Qt${QT_VERSION_MAJOR}::Core)
