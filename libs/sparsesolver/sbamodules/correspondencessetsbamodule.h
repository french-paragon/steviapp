#ifndef STEREOVISIONAPP_CORRESPONDENCESSETSBAMODULE_H
#define STEREOVISIONAPP_CORRESPONDENCESSETSBAMODULE_H

#include "../modularsbasolver.h"

#include "../../datablocks/correspondencesset.h"

namespace StereoVisionApp {

class CorrespondencesSetSBAModule : public ModularSBASolver::SBAModule
{
public:

    static const char* ModuleName;
    inline static void registerDefaultModuleFactory(SBASolverModulesInterface* interface) {
        interface->registerSBAModule(CorrespondencesSetSBAModule::ModuleName, [] (ModularSBASolver* solver) -> ModularSBASolver::SBAModule* {
            return new CorrespondencesSetSBAModule();
        });
    }

    CorrespondencesSetSBAModule();

    virtual QString moduleName() const override;

    virtual bool addGraphReductorVariables(Project *currentProject, GenericSBAGraphReductor* graphReductor) override;
    virtual bool addGraphReductorObservations(Project *currentProject, GenericSBAGraphReductor* graphReductor) override;

    virtual bool setupParameters(ModularSBASolver* solver) override;
    virtual bool init(ModularSBASolver* solver, ceres::Problem & problem) override;
    virtual bool writeResults(ModularSBASolver* solver) override;
    virtual bool writeUncertainty(ModularSBASolver* solver) override;
    virtual void cleanup(ModularSBASolver* solver) override;

    static bool addGeoPosPrior(Correspondences::Typed<Correspondences::PRIORID> const& priorId,
                               Correspondences::Typed<Correspondences::GEOXYZ> const& geoPos,
                               StereoVisionApp::ModularSBASolver* solver,
                               ceres::Problem & problem);

    static bool addGeoProjPrior(Correspondences::Typed<Correspondences::PRIORID> const& priorId,
                               Correspondences::Typed<Correspondences::GEOXY> const& geoPos,
                               StereoVisionApp::ModularSBASolver* solver,
                               ceres::Problem & problem);

    static bool addXYZMatch(Correspondences::Typed<Correspondences::XYZ> const& xyz1,
                            Correspondences::Typed<Correspondences::XYZ> const& xyz2,
                            StereoVisionApp::ModularSBASolver* solver,
                            ceres::Problem & problem);

    static bool addXYZ2LineMatch(Correspondences::Typed<Correspondences::XYZ> const& xyz,
                                 Correspondences::Typed<Correspondences::Line3D> const& line,
                                 StereoVisionApp::ModularSBASolver* solver,
                                 ceres::Problem & problem);

    static bool addXYZ2PlaneMatch(Correspondences::Typed<Correspondences::XYZ> const& xyz,
                                     Correspondences::Typed<Correspondences::Plane3D> const& line,
                                     StereoVisionApp::ModularSBASolver* solver,
                                     ceres::Problem & problem);

    static bool addXYZ2GeoMatch(Correspondences::Typed<Correspondences::XYZ> const& xyz,
                                Correspondences::Typed<Correspondences::GEOXYZ> const& geoMatch,
                                StereoVisionApp::ModularSBASolver* solver,
                                ceres::Problem & problem);

    static bool addXYZ2GeoMatch(Correspondences::Typed<Correspondences::XYZ> const& xyz,
                                Correspondences::Typed<Correspondences::GEOXY> const& geoMatch,
                                StereoVisionApp::ModularSBASolver* solver,
                                ceres::Problem & problem);

    static bool setupXYZPrior(Correspondences::Typed<Correspondences::XYZ> const& xyz,
                              StereoVisionApp::ModularSBASolver* solver,
                              ceres::Problem & problem);

    static bool addXYZ2PriorMatch(Correspondences::Typed<Correspondences::XYZ> const& xyz,
                                  Correspondences::Typed<Correspondences::PRIORID> const& priorId,
                                  StereoVisionApp::ModularSBASolver* solver,
                                  ceres::Problem & problem);

    static bool addXYZT2XYZMatch(Correspondences::Typed<Correspondences::XYZT> const& xyzt,
                                 Correspondences::Typed<Correspondences::XYZ> const& xyz,
                                 StereoVisionApp::ModularSBASolver* solver,
                                 ceres::Problem & problem);

    static bool addXYZT2XYZTMatch(Correspondences::Typed<Correspondences::XYZT> const& xyzt1,
                                  Correspondences::Typed<Correspondences::XYZT> const& xyzt2,
                                  StereoVisionApp::ModularSBASolver* solver,
                                  ceres::Problem & problem);

    static bool addUV2UVMatch(Correspondences::Typed<Correspondences::UV> const& uv1,
                              Correspondences::Typed<Correspondences::UV> const& uv2,
                              StereoVisionApp::ModularSBASolver* solver,
                              ceres::Problem & problem);

    static bool addUV2XYZMatch(Correspondences::Typed<Correspondences::UV> const& uv,
                               Correspondences::Typed<Correspondences::XYZ> const& xyz,
                               StereoVisionApp::ModularSBASolver* solver,
                               ceres::Problem & problem);

    static bool addUV2GeoXYZMatch(Correspondences::Typed<Correspondences::UV> const& uv,
                               Correspondences::Typed<Correspondences::GEOXYZ> const& xyz,
                               StereoVisionApp::ModularSBASolver* solver,
                               ceres::Problem & problem);

    static bool addUV2XYZTMatch(Correspondences::Typed<Correspondences::UV> const& uv,
                                Correspondences::Typed<Correspondences::XYZT> const& xyz,
                                StereoVisionApp::ModularSBASolver* solver,
                                ceres::Problem & problem);

    static bool addUVT2UVMatch(Correspondences::Typed<Correspondences::UVT> const& uvt,
                               Correspondences::Typed<Correspondences::UV> const& uv,
                               StereoVisionApp::ModularSBASolver* solver,
                               ceres::Problem & problem);

    static bool addUVT2UVTMatch(Correspondences::Typed<Correspondences::UVT> const& uvt1,
                               Correspondences::Typed<Correspondences::UVT> const& uvt2,
                               StereoVisionApp::ModularSBASolver* solver,
                               ceres::Problem & problem);

    static bool addUVT2XYZMatch(Correspondences::Typed<Correspondences::UVT> const& uvt,
                               Correspondences::Typed<Correspondences::XYZ> const& xyz,
                               StereoVisionApp::ModularSBASolver* solver,
                               ceres::Problem & problem);

    static bool addUVT2GeoXYZMatch(Correspondences::Typed<Correspondences::UVT> const& uvt,
                                  Correspondences::Typed<Correspondences::GEOXYZ> const& xyz,
                                  StereoVisionApp::ModularSBASolver* solver,
                                  ceres::Problem & problem);

    static bool addUVT2XYZTMatch(Correspondences::Typed<Correspondences::UVT> const& uvt,
                                Correspondences::Typed<Correspondences::XYZT> const& xyzt,
                                StereoVisionApp::ModularSBASolver* solver,
                                ceres::Problem & problem);

};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_CORRESPONDENCESSETSBAMODULE_H
