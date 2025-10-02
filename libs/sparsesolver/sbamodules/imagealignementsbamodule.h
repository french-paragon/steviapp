#ifndef STEREOVISIONAPP_IMAGEALIGNEMENTSBAMODULE_H
#define STEREOVISIONAPP_IMAGEALIGNEMENTSBAMODULE_H

#include "../modularsbasolver.h"

namespace StereoVisionApp {

class Project;
class GenericSBAGraphReductor;

class ImageAlignementSBAModule : public ModularSBASolver::SBAModule
{

public:

    static const char* ModuleName;
    inline static void registerDefaultModuleFactory(SBASolverModulesInterface* interface) {
        interface->registerSBAModule(ImageAlignementSBAModule::ModuleName, [] (ModularSBASolver* solver) -> ModularSBASolver::SBAModule* {
            return new ImageAlignementSBAModule();
        });
    }

    ImageAlignementSBAModule();

    virtual QString moduleName() const override;

    virtual bool addGraphReductorVariables(Project *currentProject, GenericSBAGraphReductor* graphReductor) override;
    virtual bool addGraphReductorObservations(Project *currentProject, GenericSBAGraphReductor* graphReductor) override;

    virtual bool setupParameters(ModularSBASolver* solver) override;
    virtual bool init(ModularSBASolver* solver, ceres::Problem & problem) override;
    virtual bool writeResults(ModularSBASolver* solver) override;
    virtual bool writeUncertainty(ModularSBASolver* solver) override;
    virtual void cleanup(ModularSBASolver* solver) override;

protected:

    //match camera internal id to a projector module for the images
    QMap<qint64, ModularSBASolver::ProjectorModule*> _cameraProjectors;

};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_IMAGEALIGNEMENTSBAMODULE_H
