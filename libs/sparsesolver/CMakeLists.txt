define_stereovisionapp_lib(SOLVING Solving)

set(LIB_SRC initialsolution.cpp
            initialsolution.h
            sbagraphreductor.cpp
            sbagraphreductor.h
            sbainitializer.cpp
            sbainitializer.h
            sparsesolverbase.cpp
			sparsesolverbase.h
			graphsbasolver.cpp
			graphsbasolver.h
			graphstereorigsolver.cpp
			graphstereorigsolver.h
            imagepointssolutionmodel.cpp
            imagepointssolutionmodel.h
            landmarkpointssolutionmodel.cpp
            landmarkpointssolutionmodel.h
            vertices/camerapose.cpp
            vertices/camerapose.h
            vertices/camparam.cpp
            vertices/camparam.h
            vertices/vertexcamerapose.cpp
            vertices/vertexcamerapose.h
            vertices/vertexcameraparam.cpp
            vertices/vertexcameraparam.h
            vertices/vertexcameraradialdistortion.cpp
            vertices/vertexcameraradialdistortion.h
            vertices/vertexcameraskewdistortion.cpp
            vertices/vertexcameraskewdistortion.h
            vertices/vertexcameratangentialdistortion.cpp
            vertices/vertexcameratangentialdistortion.h
            edges/edgeparametrizedxyz2uv.cpp
            edges/edgeparametrizedxyz2uv.h
            edges/edgese3fullprior.cpp
            edges/edgese3fullprior.h
            edges/edgese3rpyprior.cpp
            edges/edgese3rpyprior.h
            edges/edgese3xyzprior.cpp
            edges/edgese3xyzprior.h
            edges/edgexyzprior.cpp
			edges/edgexyzprior.h
			edges/edgepointdistance.cpp
			edges/edgepointdistance.h
			edges/edgepointsangle.cpp
			edges/edgepointsangle.h
            helperfunctions.h
)

add_library(
    ${SOLVING_LIB}
    ${LIB_SRC}
)

target_link_libraries(${SOLVING_LIB} PRIVATE Qt${QT_VERSION_MAJOR}::Core)
target_link_libraries(${SOLVING_LIB} PRIVATE ${DATA_LIB} ${STEREOVISION_LIB})
target_link_libraries(${SOLVING_LIB} PRIVATE ${G2O_LIBS})
