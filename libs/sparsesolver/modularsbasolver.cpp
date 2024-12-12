#include "modularsbasolver.h"

#include <sbagraphreductor.h>

#include <datablocks/project.h>
#include <datablocks/landmark.h>
#include <datablocks/image.h>
#include <datablocks/camera.h>
#include <datablocks/localcoordinatesystem.h>
#include <datablocks/trajectory.h>
#include <datablocks/datatable.h>

#include <ceres/ceres.h>

#include <ceres/cost_function.h>
#include <ceres/autodiff_cost_function.h>
#include <ceres/normal_prior.h>
#include <ceres/iteration_callback.h>

#include "./costfunctors/parametrizedxyz2uvcost.h"
#include "./costfunctors/weightedcostfunction.h"
#include "./costfunctors/interpolatedvectorprior.h"
#include "./costfunctors/leverarmcostfunctor.h"

#include <QDir>
#include <QFile>

namespace StereoVisionApp {

class ModularSBASolverIterationCallback : public ceres::IterationCallback {
public:

    ModularSBASolverIterationCallback(ModularSBASolver* solver) :
        _solver(solver)
    {

    }

    virtual ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary) override {

        if (_solver->currentStep() < _solver->optimizationSteps()-1) {
            _solver->jumpStep();
        }

        if (summary.step_is_valid) {
            return ceres::SOLVER_CONTINUE;
        }

        return ceres::SOLVER_ABORT;

    }

protected:

    ModularSBASolver* _solver;
};

ModularSBASolver::ModularSBASolver(Project *p, bool computeUncertainty, bool sparse, bool verbose, QObject *parent) :
    SparseSolverBase(p, parent),
    _compute_marginals(computeUncertainty),
    _sparse(sparse),
    _verbose(verbose),
    _loggingDir("")
{

}

ModularSBASolver::~ModularSBASolver() {

    for (SBAModule* module : _modules) {
        delete module;
    }

    for (ProjectorModule* proj : _projectors) {
        delete proj;
    }

    for (ValueBlockLogger* logger : _loggers) {
        delete logger;
    }

}

int ModularSBASolver::uncertaintySteps() const {
    return (_compute_marginals) ? 1 : 0;
}

bool ModularSBASolver::hasUncertaintyStep() const {
    return _compute_marginals;
}

bool ModularSBASolver::addModule(SBAModule* module) {

    for (SBAModule* cand : _modules) {
        if (cand == module) { //if the module already exist
            return false;
        }
    }

    _modules.push_back(module);

    return true;

}


bool ModularSBASolver::addProjector(ProjectorModule* projector) {

    if (projector == nullptr) {
        return false;
    }

    if (_projectors.contains(projector)) {
        return true;
    }

    _projectors.push_back(projector);

    return true;
}

bool ModularSBASolver::assignProjectorToFrame(ProjectorModule* projector, qint64 imId) {

    if (!_projectors.contains(projector)) {
        return false;
    }

    if (!_poseParametersIndex.contains(imId)) {
        return false;
    }

    _frameProjectorsAssociations[imId] = _projectors.indexOf(projector);

    return true;

}


ModularSBASolver::ProjectorModule* ModularSBASolver::getProjectorForFrame(qint64 imId) {

    if (_frameProjectorsAssociations.contains(imId)) {
        return _projectors[_frameProjectorsAssociations[imId]];
    }

    return nullptr;
}

ModularSBASolver::PositionNode* ModularSBASolver::getPositionNode(qint64 datablockId) {
    if (!_pointsParametersIndex.contains(datablockId)) {
        return nullptr;
    }
    return &_pointsParameters[_pointsParametersIndex[datablockId]];
}
ModularSBASolver::PoseNode* ModularSBASolver::getPoseNode(qint64 datablockId) {
    if (!_poseParametersIndex.contains(datablockId)) {
        return nullptr;
    }
    return &_poseParameters[_poseParametersIndex[datablockId]];
}

ModularSBASolver::PositionNode* ModularSBASolver::getNodeForLandmark(qint64 lmId, bool createIfMissing) {

    QTextStream out(stdout);

    if (_currentProject == nullptr) {
        return nullptr;
    }

    if (_pointsParametersIndex.contains(lmId)) {
        return &_pointsParameters[_pointsParametersIndex[lmId]];
    }

    if (!createIfMissing) {
        return nullptr;
    }

    Landmark* lm = _currentProject->getDataBlock<Landmark>(lmId);

    if (lm == nullptr) {
        return nullptr;
    }

    if (!lm->isEnabled()) {
        return nullptr;
    }

    int idx = _pointsParameters.size();
    _pointsParameters.emplace_back();
    _pointsParametersIndex[lmId] = idx;

    ModularSBASolver::PositionNode* lmNode = &_pointsParameters[idx];
    lmNode->datablockId = lmId;

    return lmNode;

}

ModularSBASolver::PoseNode* ModularSBASolver::getNodeForFrame(qint64 imId, bool createIfMissing) {

    QTextStream out(stdout);

    if (_currentProject == nullptr) {
        return nullptr;
    }

    if (_poseParametersIndex.contains(imId)) {
        return &_poseParameters[_poseParametersIndex[imId]];
    }

    if (!createIfMissing) {
        return nullptr;
    }

    Image* im = _currentProject->getDataBlock<Image>(imId);

    if (im == nullptr) {
        return nullptr;
    }

    if (!im->isEnabled()) {
        return nullptr;
    }

    int idx = _poseParameters.size();
    _poseParameters.emplace_back();
    _poseParametersIndex[imId] = idx;
    ModularSBASolver::PoseNode* imNode = &_poseParameters[idx];
    imNode->datablockId = imId;

    imNode->rAxis[0] = im->optRot().value(0);
    imNode->rAxis[1] = im->optRot().value(1);
    imNode->rAxis[2] = im->optRot().value(2);

    imNode->t[0] = im->optPos().value(0);
    imNode->t[1] = im->optPos().value(1);
    imNode->t[2] = im->optPos().value(2);

    std::array<double, 3> raxis_prior;
    std::array<double, 3> t_prior;

    //TODO: check if the use of axis angle representation here is consistent.
    if (im->xRot().isSet() and im->yRot().isSet() and im->zRot().isSet()) {
        raxis_prior[0] = im->xRot().value();
        raxis_prior[1] = im->yRot().value();
        raxis_prior[2] = im->zRot().value();
    }

    if (im->xCoord().isSet() and im->yCoord().isSet() and im->zCoord().isSet()) {
        t_prior[0] = im->xCoord().value();
        t_prior[1] = im->yCoord().value();
        t_prior[2] = im->zCoord().value();
    }

    if (im->isFixed()) {

        imNode->rAxis = raxis_prior;
        imNode->t = t_prior;
    }

    return &_poseParameters[_poseParametersIndex[imId]];

}

ModularSBASolver::PoseNode* ModularSBASolver::getNodeForLocalCoordinates(qint64 lcsId, bool createIfMissing) {

    if (_currentProject == nullptr) {
        return nullptr;
    }

    if (_poseParametersIndex.contains(lcsId)) {
        return &_poseParameters[_poseParametersIndex[lcsId]];
    }

    if (!createIfMissing) {
        return nullptr;
    }

    LocalCoordinateSystem* lcs = _currentProject->getDataBlock<LocalCoordinateSystem>(lcsId);

    if (lcs == nullptr) {
        return nullptr;
    }

    int idx = _poseParameters.size();
    _poseParameters.emplace_back();
    _poseParametersIndex[lcsId] = idx;

    PoseNode& node = _poseParameters[idx];

    _poseParameters[idx].datablockId = lcs->internalId();

    node.rAxis[0] = lcs->optRot().value(0);
    node.rAxis[1] = lcs->optRot().value(1);
    node.rAxis[2] = lcs->optRot().value(2);

    node.t[0] = lcs->optPos().value(0);
    node.t[1] = lcs->optPos().value(1);
    node.t[2] = lcs->optPos().value(2);

    std::array<double, 3> raxis_prior;
    std::array<double, 3> t_prior;

    //TODO: check if the use of axis angle representation here is consistent.
    if (lcs->xRot().isSet() and lcs->yRot().isSet() and lcs->zRot().isSet()) {
        raxis_prior[0] = lcs->xRot().value();
        raxis_prior[1] = lcs->yRot().value();
        raxis_prior[2] = lcs->zRot().value();
    }

    if (lcs->xCoord().isSet() and lcs->yCoord().isSet() and lcs->zCoord().isSet()) {
        t_prior[0] = lcs->xCoord().value();
        t_prior[1] = lcs->yCoord().value();
        t_prior[2] = lcs->zCoord().value();
    }

    if (lcs->isFixed()) {

        node.rAxis = raxis_prior;
        node.t = t_prior;
    }

    Trajectory* traj = lcs->getAssignedTrajectory();

    if (traj != nullptr) {
        node.trajectoryId = traj->internalId();
    } else {
        node.trajectoryId = std::nullopt;
    }

    return &_poseParameters[idx];

}

ModularSBASolver::TrajectoryNode* ModularSBASolver::getNodeForTrajectory(qint64 trajId, bool createIfMissing) {

    if (_currentProject == nullptr) {
        return nullptr;
    }

    if (_trajectoryParametersIndex.contains(trajId)) {
        return &_trajectoryParameters[_trajectoryParametersIndex[trajId]];
    }

    if (!createIfMissing) {
        return nullptr;
    }

    Trajectory* lcs = _currentProject->getDataBlock<Trajectory>(trajId);

    if (lcs == nullptr) {
        return nullptr;
    }

    int idx = _trajectoryParameters.size();
    _trajectoryParameters.emplace_back();
    _trajectoryParametersIndex[trajId] = idx;
    _trajectoryParameters[idx].trajId = trajId;
    return &_trajectoryParameters[idx];

}

ModularSBASolver::LeverArmNode* ModularSBASolver::getNodeForLeverArm(QPair<qint64,qint64> camtrajId, bool createIfMissing) {

    if (_currentProject == nullptr) {
        return nullptr;
    }

    if (_leverArmParametersIndex.contains(camtrajId)) {
        return &_leverArmParameters[_leverArmParametersIndex[camtrajId]];
    }

    if (!createIfMissing) {
        return nullptr;
    }

    int idx = _leverArmParameters.size();
    _leverArmParameters.emplace_back();

    _leverArmParametersIndex[camtrajId] = idx;

    _leverArmParameters[idx].PlatformId = camtrajId.first;
    _leverArmParameters[idx].TrajId = camtrajId.second;

    //init lever arm to be 0.
    for (int i = 0; i < 3; i++) {
        _leverArmParameters[idx].t[i] = 0;
        _leverArmParameters[idx].rAxis[i] = 0;
    }

    _problem.AddParameterBlock(_leverArmParameters[idx].t.data(), _leverArmParameters[idx].t.size());
    _problem.AddParameterBlock(_leverArmParameters[idx].rAxis.data(), _leverArmParameters[idx].rAxis.size());

    return &_leverArmParameters[idx];

}

StereoVision::Geometry::AffineTransform<double> ModularSBASolver::getTransform2LocalFrame() const {

    if (_currentProject != nullptr) {
        if (_currentProject->hasLocalCoordinateFrame()) {
            return _currentProject->ecef2local().cast<double>();
        }
    }

    return StereoVision::Geometry::AffineTransform<double>(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());

}

void ModularSBASolver::enableLogging(QString loggingDirPath) {
    _loggingDir = loggingDirPath;
    _default_logging_file = QDir(_loggingDir).absoluteFilePath("messages.log");
}
void ModularSBASolver::logDatas(QString fileName) {

    if (_loggingDir.isEmpty() or _loggers.isEmpty()) {
        return;
    }

    QDir loggingDir (_loggingDir);

    if (!loggingDir.exists()) {
        bool ok = loggingDir.mkpath(".");

        if (!ok) {
            return;
        }
    }

    QString errorsFilePath = loggingDir.filePath(fileName);

    QFile errorsFile(errorsFilePath);

    bool ok = errorsFile.open(QFile::WriteOnly);

    if (!ok) {
        return;
    }

    QTextStream errorStream(&errorsFile);

    QStringList loggersKeys = _loggers.keys();

    for (QString loggerKey : loggersKeys) {
        errorStream << loggerKey << ": ";
        _loggers[loggerKey]->log(errorStream);
        errorStream << "\n";
    }

    errorStream.flush();

}

void ModularSBASolver::logMessage(QString message) {

    QTextStream err(stderr);

    if (_loggingDir.isEmpty()) {
        //logging disabled
        return;
    }

    QFile logFile(_default_logging_file);

    bool status = logFile.open(QFile::WriteOnly|QFile::Append);

    if (!status) {
        err << "Could not open log file << \"" << _default_logging_file << "\"" << Qt::endl;
        return;
    }

    QTextStream log(&logFile);

    QDateTime now = QDateTime::currentDateTimeUtc();

    log << now.toString() << " UTC " << message << Qt::endl;

    logFile.close();

}

void ModularSBASolver::addLogger(QString const& loggerName, ValueBlockLogger* logger) {
    if (_loggers.values().contains(logger)) {
        return;
    }

    if (_loggers.contains(loggerName)) {
        delete _loggers[loggerName];
        _loggers.remove(loggerName);
    }

    if (logger != nullptr) {
        _loggers.insert(loggerName, logger);
    }
}

bool ModularSBASolver::itemIsObservable(qint64 itemId) const {
    return _observabilityGraph.isIdConstrained(itemId);
}

bool ModularSBASolver::init() {

    if (_currentProject == nullptr) {
        return false;
    }

    //reserve enough memory so the objects are not moved.
    //TODO: create a vector class which garantee that the allocated chunk is not moved unless the vector is reset.
    _pointsParameters.reserve(_currentProject->countTypeInstances(Landmark::staticMetaObject.className()));
    int nPoses = _currentProject->countTypeInstances(Image::staticMetaObject.className());
    nPoses += _currentProject->countTypeInstances(LocalCoordinateSystem::staticMetaObject.className());
    _poseParameters.reserve(nPoses);
    _trajectoryParameters.reserve(_currentProject->countTypeInstances(Trajectory::staticMetaObject.className()));
    _leverArmParameters.reserve(_trajectoryParameters.size()*_currentProject->countTypeInstances(Camera::staticMetaObject.className()));

    //reset the problem and observability graph
    _observabilityGraph.clear();
    _problem = ceres::Problem();

    bool ok = true;

    for (SBAModule* module : _modules) {
        ok = module->addGraphReductorVariables(_currentProject, &_observabilityGraph);

        if (!ok) {
            return false;
        }
    }

    for (SBAModule* module : _modules) {
        ok = module->addGraphReductorObservations(_currentProject, &_observabilityGraph);

        if (!ok) {
            return false;
        }
    }

    _observabilityGraph.reduceGraph();

    for (SBAModule* module : _modules) {
        ok = module->setupParameters(this);

        if (!ok) {
            return false;
        }
    }

    ok = initManagedParameters();

    if (!ok) {
        return false;
    }

    //setup the projectors
    for (ProjectorModule* module : _projectors) {
        ok = module->init(this, _problem);

        if (!ok) {
            return false;
        }
    }

    //setup the sba modules
    for (SBAModule* module : _modules) {
        ok = module->init(this, _problem);

        if (!ok) {
            return false;
        }
    }

    logDatas("after_init.log");
    return true;
}

bool ModularSBASolver::initManagedParameters() {

    if(_currentProject == nullptr) {
        return false;
    }

    for (PositionNode& node : _pointsParameters) {

        DataBlock* block = _currentProject->getById(node.datablockId);

        if (block == nullptr) {
            continue;
        }

        _problem.AddParameterBlock(node.pos.data(), node.pos.size());

        if (block->isFixed()) {
            _problem.SetParameterBlockConstant(node.pos.data());
        }
    }

    for (PoseNode& node : _poseParameters) {

        DataBlock* block = _currentProject->getById(node.datablockId);

        if (block == nullptr) {
            continue;
        }

        _problem.AddParameterBlock(node.rAxis.data(), node.rAxis.size());
        _problem.AddParameterBlock(node.t.data(), node.t.size());

        if (block->isFixed()) {
            _problem.SetParameterBlockConstant(node.rAxis.data());
            _problem.SetParameterBlockConstant(node.t.data());
        }
    }

    for (LeverArmNode& node : _leverArmParameters) {

        DataBlock* block = _currentProject->getById(node.PlatformId);

        if (block == nullptr) {
            continue;
        }

        _problem.AddParameterBlock(node.rAxis.data(), node.rAxis.size());
        _problem.AddParameterBlock(node.t.data(), node.t.size());

        if (block->isFixed()) {
            _problem.SetParameterBlockConstant(node.rAxis.data());
            _problem.SetParameterBlockConstant(node.t.data());
        }
    }

    return true;

}

bool ModularSBASolver::opt_step() {

    if (_not_first_step) {
        return true; //optimization already converged
    }

    if (_verbose) {
        std::cout << "Start optimization!" << std::endl;
    }

    ceres::Solver::Options options;
    options.linear_solver_type = (_sparse) ? ceres::SPARSE_SCHUR : ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = _verbose;

    ModularSBASolverIterationCallback* iterationCallBack = new ModularSBASolverIterationCallback(this);

    options.max_num_iterations = optimizationSteps();

    options.callbacks = {iterationCallBack};

    ceres::Solver::Summary summary;

    ceres::Solve(options, &_problem, &summary);

    _not_first_step = true;

    return true;

}
bool ModularSBASolver::std_step() {
    //TODO: find how the modules request which parameters uncertainty they need.
    return true; //TODO: implement the stochastic approach we proposed

}
bool ModularSBASolver::writeResults() {

    if (_currentProject == nullptr) {
        return false;
    }

    bool ok = true;

    logDatas("after_opt.log");

    for (SBAModule* module : _modules) {
        ok = module->writeResults(this);

        if (!ok) {
            return false;
        }
    }

    for (ProjectorModule* projector : _projectors) {
        ok = projector->writeResults(this);

        if (!ok) {
            return false;
        }
    }

    return true;

}
bool ModularSBASolver::writeUncertainty() {
    //todo, find a mechanism for the modules to write uncertainty
    return true;
}
void ModularSBASolver::cleanup() {

    for (SBAModule* module : _modules) {
        module->cleanup(this);
    }

    for (ProjectorModule* projector : _projectors) {
        projector->cleanup(this);
    }

    //clear the basic nodes
    _pointsParameters.clear();
    _pointsParametersIndex.clear();

    _poseParameters.clear();
    _poseParametersIndex.clear();

    _trajectoryParameters.clear();
    _trajectoryParametersIndex.clear();

    _leverArmParameters.clear();
    _leverArmParametersIndex.clear();

    _problem = ceres::Problem(); //reset problem
}

bool ModularSBASolver::splitOptSteps() const {
    return true;
}

ModularSBASolver::SBAModule::SBAModule() {

}
ModularSBASolver::SBAModule::~SBAModule() {

}

ModularSBASolver::ProjectorModule::ProjectorModule() {

}
ModularSBASolver::ProjectorModule::~ProjectorModule() {

}
ModularSBASolver::ValueBlockLogger::ValueBlockLogger() {

}
ModularSBASolver::ValueBlockLogger::~ValueBlockLogger() {

}
QTextStream& ModularSBASolver::ValueBlockLogger::log(QTextStream& stream) const {
    QVector<double> errors = getErrors();

    for (double& err : errors) {
        stream << ' ' << err;
    }

    return stream;
}


const char* SBASolverModulesInterface::AppInterfaceName = "StereoVisionApp::SBASolverModulesInterface";

SBASolverModulesInterface::SBASolverModulesInterface(QObject* parent) :
    QObject(parent)
{

}

} // namespace StereoVisionApp
