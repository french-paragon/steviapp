#ifndef STEREOVISIONAPP_LANDMARKSSBAMODULE_H
#define STEREOVISIONAPP_LANDMARKSSBAMODULE_H

#include "../modularsbasolver.h"

namespace StereoVisionApp {

class LandmarksSBAModule : public ModularSBASolver::SBAModule
{
public:

    static const char* ModuleName;
    inline static void registerDefaultModuleFactory(SBASolverModulesInterface* interface) {
        interface->registerSBAModule(LandmarksSBAModule::ModuleName, [] (ModularSBASolver* solver) -> ModularSBASolver::SBAModule* {
            return new LandmarksSBAModule();
        });
    }


    LandmarksSBAModule();

    virtual QString moduleName() const override;

    virtual bool addGraphReductorVariables(Project *currentProject, GenericSBAGraphReductor* graphReductor) override;
    virtual bool addGraphReductorObservations(Project *currentProject, GenericSBAGraphReductor* graphReductor) override;

    virtual bool setupParameters(ModularSBASolver* solver) override;
    virtual bool init(ModularSBASolver* solver, ceres::Problem & problem) override;
    virtual bool writeResults(ModularSBASolver* solver) override;
    virtual std::vector<std::pair<const double*, const double*>> requestUncertainty(ModularSBASolver* solver, ceres::Problem & problem) override;
    virtual bool writeUncertainty(ModularSBASolver* solver) override;
    virtual void cleanup(ModularSBASolver* solver) override;

protected :

    bool initCorrespondencesSetsConstraints(StereoVisionApp::ModularSBASolver* solver, ceres::Problem & problem);

};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_LANDMARKSSBAMODULE_H
