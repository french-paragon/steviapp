define_stereovisionapp_lib(SOLVING Solving)

set(LIB_SRC initialsolution.cpp
            sbagraphreductor.cpp
            sbainitializer.cpp
            sparsesolverbase.cpp
            graphroughbundleadjustementsolver.cpp
            graphsbasolver.cpp
            ceressbasolver.cpp
            modularsbasolver.cpp
            graphstereorigsolver.cpp
            graphcameracalibrator.cpp
            imagepointssolutionmodel.cpp
            landmarkpointssolutionmodel.cpp
            localcoordinatespointssolutionmodel.cpp
            vertices/camerapose.cpp
            vertices/camparam.cpp
            vertices/vertexrigidbodypose.cpp
            vertices/vertexcamerapose.cpp
            vertices/vertexcameraparam.cpp
            vertices/vertexcameraradialdistortion.cpp
            vertices/vertexcameraskewdistortion.cpp
            vertices/vertexcameratangentialdistortion.cpp
            vertices/vertexlandmarkpos.cpp
            edges/edgeparametrizedxyz2uv.cpp
            edges/edgese3fullprior.cpp
            edges/edgese3rpyprior.cpp
            edges/edgese3xyzprior.cpp
            edges/edgexyzprior.cpp
            edges/edgepointdistance.cpp
            edges/edgepointsangle.cpp
            edges/edgecameralandmarkdirectionalalignement.cpp
            edges/edgecamerase3leverarm.cpp
            edges/edgelocalpointcoordinates.cpp
            costfunctors/leverarmcostfunctor.cpp
            costfunctors/parametrizedxyz2uvcost.cpp
            costfunctors/localpointalignementcost.cpp
            costfunctors/camposwithparametrizedleverarmcost.cpp
            costfunctors/weightedcostfunction.cpp
            costfunctors/imustepcost.cpp
            costfunctors/local3dcoalignementcost.cpp
            costfunctors/local3dtoimageuvcost.cpp
            costfunctors/interpolatedvectorprior.cpp
            sbamodules/trajectorybasesbamodule.cpp
)

set(LIB_HEADERS
    initialsolution.h
    sbagraphreductor.h
    sbainitializer.h
    graphsbasolver.h
    ceressbasolver.h
    modularsbasolver.h
    fixedpreoptimizedparameters.h
    landmarkpointssolutionmodel.h
    localcoordinatespointssolutionmodel.h
    graphroughbundleadjustementsolver.h
    sparsesolverbase.h
    graphcameracalibrator.h
    graphstereorigsolver.h
    imagepointssolutionmodel.h
    helperfunctions.h
)

set(LIB_HEADERS_EDGES
    edges/edgelocalpointcoordinates.h
    edges/edgecamerase3leverarm.h
    edges/edgecameralandmarkdirectionalalignement.h
    edges/edgepointsangle.h
    edges/edgepointdistance.h
    edges/edgexyzprior.h
    edges/edgese3xyzprior.h
    edges/edgese3rpyprior.h
    edges/edgese3fullprior.h
    edges/edgeparametrizedxyz2uv.h
)

set(LIB_HEADERS_VERTICES
    vertices/vertexlandmarkpos.h
    vertices/vertexcameratangentialdistortion.h
    vertices/vertexcameraskewdistortion.h
    vertices/vertexcameraradialdistortion.h
    vertices/vertexcameraparam.h
    vertices/vertexcamerapose.h
    vertices/vertexrigidbodypose.h
    vertices/camparam.h
    vertices/camerapose.h
)

set(LIB_HEADERS_COST_FUNCTORS
    costfunctors/leverarmcostfunctor.h
    costfunctors/camposwithparametrizedleverarmcost.h
    costfunctors/parametrizedxyz2uvcost.h
    costfunctors/localpointalignementcost.h
    costfunctors/weightedcostfunction.h
    costfunctors/imustepcost.h
    costfunctors/local3dcoalignementcost.h
    costfunctors/local3dtoimageuvcost.h
    costfunctors/interpolatedvectorprior.h
)

set(LIB_HEADERS_SBA_MODULES
    sbamodules/trajectorybasesbamodule.h
)

add_library(
    ${SOLVING_LIB}
    ${LIB_SRC}
    ${LIB_HEADERS}
    ${LIB_HEADERS_EDGES}
    ${LIB_HEADERS_VERTICES}
    ${LIB_HEADERS_COST_FUNCTORS}
    ${LIB_HEADERS_SBA_MODULES}
)

target_link_libraries(${SOLVING_LIB} PRIVATE Qt${QT_VERSION_MAJOR}::Core)
target_link_libraries(${SOLVING_LIB} PRIVATE ${DATA_LIB} ${PROCESSING_LIB} StereoVision::stevi)
target_link_libraries(${SOLVING_LIB} PRIVATE ${G2O_LIBS} ${CERES_LIBRARIES} ${CSPARSE_LIBRARY})

install_stereovisionapp_sdklib(${SOLVING_LIB})

get_filename_component(LibDirName ${CMAKE_CURRENT_SOURCE_DIR} NAME)

install(FILES ${LIB_HEADERS}
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${LibDirName}
  )

install(FILES ${LIB_HEADERS_EDGES}
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${LibDirName}/edges
  )

install(FILES ${LIB_HEADERS_VERTICES}
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${LibDirName}/vertices
  )

install(FILES ${LIB_HEADERS_COST_FUNCTORS}
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${LibDirName}/costfunctors
  )

install(FILES ${LIB_HEADERS_SBA_MODULES}
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${LibDirName}/sbamodules
  )
