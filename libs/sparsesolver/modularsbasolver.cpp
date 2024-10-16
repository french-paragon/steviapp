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

    if (!_frameParametersIndex.contains(imId)) {
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

ModularSBASolver::LandmarkNode* ModularSBASolver::getNodeForLandmark(qint64 lmId, bool createIfMissing) {

    QTextStream out(stdout);

    if (_currentProject == nullptr) {
        return nullptr;
    }

    if (_LandmarkParametersIndex.contains(lmId)) {
        return &_LandmarkParameters[_LandmarkParametersIndex[lmId]];
    }

    if (!createIfMissing) {
        return nullptr;
    }

    Landmark* lm = _currentProject->getDataBlock<Landmark>(lmId);

    if (lm == nullptr) {
        return nullptr;
    }

    int idx = _LandmarkParameters.size();
    _LandmarkParameters.emplace_back();
    _LandmarkParametersIndex[lmId] = idx;

    ModularSBASolver::LandmarkNode* lmNode = &_LandmarkParameters[idx];
    lmNode->lmId = lmId;

    _problem.AddParameterBlock(lmNode->pos.data(), lmNode->pos.size());

    QString paramLogName = QString("Position for landmark %1").arg(lm->objectName());

    addLogger(paramLogName, new ParamsValsLogger<3>(lmNode->pos.data()));

    constexpr bool optimized = true;
    constexpr bool notOptimized = false;

    //get the initial optimization value in optimization frame (ecef for georeferenced).
    std::optional<Eigen::Vector3f> coordInitial = lm->getOptimizableCoordinates(optimized);

    if (!coordInitial.has_value()) {
        coordInitial = lm->getOptimizableCoordinates(notOptimized);
    }

    Eigen::Vector3d vecInitial;

    if (coordInitial.has_value()) {
        vecInitial = coordInitial.value().cast<double>();
    }

    //move to local optimization frame
    vecInitial = getTransform2LocalFrame()*vecInitial;

    if (!coordInitial.has_value()) {
        vecInitial = Eigen::Vector3d::Zero();
    }

    lmNode->pos[0] = vecInitial.x();
    lmNode->pos[1] = vecInitial.y();
    lmNode->pos[2] = vecInitial.z();

    std::optional<Eigen::Vector3f> coordPrior = lm->getOptimizableCoordinates(notOptimized);

    if (coordPrior.has_value()) {

        Eigen::Vector3d vecPrior = coordPrior.value().cast<double>();

        vecPrior = getTransform2LocalFrame()*vecPrior;

        ceres::Vector m;
        m.resize(3);

        m[0] = vecPrior.x();
        m[1] = vecPrior.y();
        m[2] = vecPrior.z();

        Eigen::Matrix3d stiffness = Eigen::Matrix3d::Identity();

        if (lm->xCoord().isUncertain()) {
            stiffness(0,0) = 1./std::abs(lm->xCoord().stddev());
        } else {
            stiffness(0,0) = 1e6;
        }

        if (lm->yCoord().isUncertain()) {
            stiffness(1,1) = 1./std::abs(lm->yCoord().stddev());
        } else {
            stiffness(1,1) = 1e6;
        }

        if (lm->zCoord().isUncertain()) {
            stiffness(2,2) = 1./std::abs(lm->zCoord().stddev());
        } else {
            stiffness(2,2) = 1e6;
        }

        ceres::NormalPrior* normalPrior = new ceres::NormalPrior(stiffness, m);

        _problem.AddResidualBlock(normalPrior, nullptr, lmNode->pos.data());

        if (_verbose) {

            std::array<double,3> res;

            out << "Position prior (lm " << lmId << ") ";
            out << "initial residuals = [" << lmNode->pos[0] - m[0] << " " << lmNode->pos[1] - m[1] << " " << lmNode->pos[2] - m[2] << "]\n";
        }
    }

    return lmNode;

}

ModularSBASolver::PoseNode* ModularSBASolver::getNodeForFrame(qint64 imId, bool createIfMissing) {

    QTextStream out(stdout);

    if (_currentProject == nullptr) {
        return nullptr;
    }

    if (_frameParametersIndex.contains(imId)) {
        return &_frameParameters[_frameParametersIndex[imId]];
    }

    if (!createIfMissing) {
        return nullptr;
    }

    Image* im = _currentProject->getDataBlock<Image>(imId);

    if (im == nullptr) {
        return nullptr;
    }

    int idx = _frameParameters.size();
    _frameParameters.emplace_back();
    _frameParametersIndex[imId] = idx;
    ModularSBASolver::PoseNode* imNode = &_frameParameters[idx];
    imNode->frameId = imId;

    imNode->rAxis[0] = im->optRot().value(0);
    imNode->rAxis[1] = im->optRot().value(1);
    imNode->rAxis[2] = im->optRot().value(2);

    imNode->t[0] = im->optPos().value(0);
    imNode->t[1] = im->optPos().value(1);
    imNode->t[2] = im->optPos().value(2);

    _problem.AddParameterBlock(imNode->rAxis.data(), imNode->rAxis.size());
    _problem.AddParameterBlock(imNode->t.data(), imNode->t.size());

    if (getFixedParametersFlag()&FixedParameter::CameraExternal) {
        _problem.SetParameterBlockConstant(imNode->rAxis.data());
        _problem.SetParameterBlockConstant(imNode->t.data());
    }

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

        _problem.SetParameterBlockConstant(imNode->rAxis.data());
        _problem.SetParameterBlockConstant(imNode->t.data());

    }

    if (!im->isFixed() and im->xCoord().isUncertain() and im->yCoord().isUncertain() and im->zCoord().isUncertain()) {

        ceres::Matrix At;
        At.setConstant(3,3,0);

        At(0,0) = 1./std::abs(im->xCoord().stddev());
        At(1,1) = 1./std::abs(im->yCoord().stddev());
        At(2,2) = 1./std::abs(im->zCoord().stddev());

        ceres::Vector bt;
        bt.resize(3);

        bt << t_prior[0],t_prior[1], t_prior[2];

        ceres::NormalPrior* tPrior = new ceres::NormalPrior(At, bt);
        _problem.AddResidualBlock(tPrior, nullptr, imNode->t.data());

        if (_verbose) {

            std::array<double,3> res;

            out << "Position prior (img " << imId << ") ";
            out << "initial residuals = [" << imNode->t[0] - t_prior[0] << " " << imNode->t[1] - t_prior[1] << " " << imNode->t[2] - t_prior[2] << "]\n";
        }

    }

    if (!im->isFixed() and im->xRot().isUncertain() and im->yRot().isUncertain() and im->zRot().isUncertain()) {

        ceres::Matrix Araxis;
        Araxis.setConstant(3, 3, 0);

        Araxis(0,0) = 1./std::abs(im->xRot().stddev());
        Araxis(1,1) = 1./std::abs(im->yRot().stddev());
        Araxis(2,2) = 1./std::abs(im->zRot().stddev());

        ceres::Vector braxis;
        braxis.resize(3);

        braxis << raxis_prior[0],raxis_prior[1], raxis_prior[2];

        ceres::NormalPrior* rAxisPrior = new ceres::NormalPrior(Araxis, (braxis));
        _problem.AddResidualBlock(rAxisPrior, nullptr, imNode->rAxis.data());

        if (_verbose) {

            std::array<double,3> res;

            out << "Rotation axis prior (img " << imId << ") ";
            out << "initial residuals = [" << imNode->rAxis[0] - raxis_prior[0] << " " << imNode->rAxis[1] - raxis_prior[1] << " " << imNode->rAxis[2] - raxis_prior[2] << "]\n";
        }

    }

    return &_frameParameters[_frameParametersIndex[imId]];

}

ModularSBASolver::PoseNode* ModularSBASolver::getNodeForLocalCoordinates(qint64 lcsId, bool createIfMissing) {

    if (_currentProject == nullptr) {
        return nullptr;
    }

    if (_localCoordinatesParametersIndex.contains(lcsId)) {
        return &_localCoordinatesParameters[_localCoordinatesParametersIndex[lcsId]];
    }

    if (!createIfMissing) {
        return nullptr;
    }

    LocalCoordinateSystem* lcs = _currentProject->getDataBlock<LocalCoordinateSystem>(lcsId);

    if (lcs == nullptr) {
        return nullptr;
    }

    int idx = _localCoordinatesParameters.size();
    _localCoordinatesParameters.emplace_back();
    _localCoordinatesParametersIndex[lcsId] = idx;



    _problem.AddParameterBlock(_localCoordinatesParameters[idx].t.data(),
                               _localCoordinatesParameters[idx].t.size());
    _problem.AddParameterBlock(_localCoordinatesParameters[idx].rAxis.data(),
                               _localCoordinatesParameters[idx].rAxis.size());

    return &_localCoordinatesParameters[idx];

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
    _LandmarkParameters.reserve(_currentProject->countTypeInstances(Landmark::staticMetaObject.className()));
    _frameParameters.reserve(_currentProject->countTypeInstances(Image::staticMetaObject.className()));
    _localCoordinatesParameters.reserve(_currentProject->countTypeInstances(LocalCoordinateSystem::staticMetaObject.className()));
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
        ok = module->init(this, _problem);

        if (!ok) {
            return false;
        }
    }

    logDatas("after_init.log");
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

    StereoVision::Geometry::AffineTransform<double> ecef2local = getTransform2LocalFrame();

    StereoVision::Geometry::AffineTransform<double>
            local2ecef(ecef2local.R.transpose(),
                       -ecef2local.R.transpose()*ecef2local.t);

    for (qint64 id : _LandmarkParametersIndex.keys()) {

        Landmark* lm = qobject_cast<Landmark*>(_currentProject->getById(id));

        if (lm->isFixed()) {
            continue;
        }

        LandmarkNode& lm_p = _LandmarkParameters[_LandmarkParametersIndex[id]];

        if (lm->geoReferenceSupportActive()) {

            Eigen::Vector3d pos;

            pos[0] = lm_p.pos[0];
            pos[1] = lm_p.pos[1];
            pos[2] = lm_p.pos[2];

            Eigen::Vector3d tmp = local2ecef*pos;
            pos = tmp;

            constexpr bool optimized = true;
            lm->setPositionFromEcef(pos.cast<float>(), optimized);

        } else {

            floatParameterGroup<3> pos;
            pos.value(0) = static_cast<float>(lm_p.pos[0]);
            pos.value(1) = static_cast<float>(lm_p.pos[1]);
            pos.value(2) = static_cast<float>(lm_p.pos[2]);
            pos.setIsSet();
            lm->setOptPos(pos);
        }

    }

    for (qint64 id : _frameParametersIndex.keys()) {

        Image* im = qobject_cast<Image*>(_currentProject->getById(id));

        if (im->isFixed()) {
            continue;
        }

        PoseNode& im_p = _frameParameters[_frameParametersIndex[id]];

        floatParameterGroup<3> pos;
        pos.value(0) = static_cast<float>(im_p.t[0]);
        pos.value(1) = static_cast<float>(im_p.t[1]);
        pos.value(2) = static_cast<float>(im_p.t[2]);
        pos.setIsSet();
        im->setOptPos(pos);

        floatParameterGroup<3> rot;
        rot.value(0) = static_cast<float>(im_p.rAxis[0]);
        rot.value(1) = static_cast<float>(im_p.rAxis[1]);
        rot.value(2) = static_cast<float>(im_p.rAxis[2]);
        rot.setIsSet();
        im->setOptRot(rot);

    }

    for (qint64 id : _localCoordinatesParametersIndex.keys()) {

        LocalCoordinateSystem* lcs = _currentProject->getDataBlock<LocalCoordinateSystem>(id);

        if (lcs->isFixed()) {
            continue;
        }

        PoseNode& lcr_p = _localCoordinatesParameters[_localCoordinatesParametersIndex[id]];

        floatParameterGroup<3> pos;
        pos.value(0) = static_cast<float>(lcr_p.t[0]);
        pos.value(1) = static_cast<float>(lcr_p.t[1]);
        pos.value(2) = static_cast<float>(lcr_p.t[1]);
        pos.setIsSet();
        lcs->setOptPos(pos);

        floatParameterGroup<3> rot;
        rot.value(0) = static_cast<float>(lcr_p.rAxis[0]);
        rot.value(1) = static_cast<float>(lcr_p.rAxis[1]);
        rot.value(2) = static_cast<float>(lcr_p.rAxis[2]);
        rot.setIsSet();
        lcs->setOptRot(rot);

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
    _LandmarkParametersIndex.clear();
    _frameParametersIndex.clear();
    _localCoordinatesParametersIndex.clear();
    _trajectoryParametersIndex.clear();

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

const char* ImageAlignementSBAModule::ModuleName = "SBAModule::ImageAlignement";

ImageAlignementSBAModule::ImageAlignementSBAModule()
{

}

bool ImageAlignementSBAModule::addGraphReductorVariables(Project *currentProject, GenericSBAGraphReductor* graphReductor) {

    if (currentProject == nullptr) {
        return false;
    }

    QVector<qint64> imgs_v = currentProject->getIdsByClass(ImageFactory::imageClassName());
    QVector<qint64> lmks_v = currentProject->getIdsByClass(LandmarkFactory::landmarkClassName());

    for (qint64 id : imgs_v) {
        graphReductor->insertItem(id, 6);
    }

    for (qint64 id : lmks_v) {
        graphReductor->insertItem(id, 3);
    }

    return true;

}
bool ImageAlignementSBAModule::addGraphReductorObservations(Project *currentProject, GenericSBAGraphReductor* graphReductor) {

    if (currentProject == nullptr) {
        return false;
    }

    QVector<qint64> imgs_v = currentProject->getIdsByClass(ImageFactory::imageClassName());

    for (qint64 id : imgs_v) {
        Image* im = qobject_cast<Image*>(currentProject->getById(id));
        if (im == nullptr) {
            continue;
        }

        if (im->isEnabled() == false) {
            continue;
        }

        int nSelfObs = 0;

        if (im->xCoord().isSet() and im->yCoord().isSet() and im->zCoord().isSet()) {
            nSelfObs += 3;
        }

        if(im->xRot().isSet() and im->yRot().isSet() and im->zRot().isSet()) {
            nSelfObs += 3;
        }

        graphReductor->insertSelfObservation(id, nSelfObs);

        QVector<qint64> connections = im->getAttachedLandmarksIds();

        for (qint64 lmId : connections) {
            graphReductor->insertObservation(id, lmId, 2);
        }

        qint64 trajId = im->assignedTrajectory(); //if the image has a trajectory

        if (trajId >= 0) {
            graphReductor->insertObservation(id, trajId, 6);
        }
    }

    return true;

}

bool ImageAlignementSBAModule::init(ModularSBASolver* solver, ceres::Problem & problem) {

    Project* currentProject = solver->currentProject();

    if (currentProject == nullptr) {
        return false;
    }

    SBAGraphReductor selector(3,2,true,true);

    SBAGraphReductor::elementsSet selection = selector(currentProject, false);

    if (selection.imgs.isEmpty() or selection.pts.isEmpty()) {
        return false;
    }

    //add images and points

    for (qint64 imId : selection.imgs) {

        Image* im = qobject_cast<Image*>(currentProject->getById(imId));

        if (im == nullptr) {
            continue;
        }

        ModularSBASolver::PoseNode* imNode = solver->getNodeForFrame(imId, true);

        if (imNode == nullptr) {
            continue;
        }

        //check if a previous module assigned a projection module already to the frame.
        ModularSBASolver::ProjectorModule* projectionModule = solver->getProjectorForFrame(imId);

        if (projectionModule == nullptr) {

            Camera* cam = im->getAssignedCamera();

            if (cam == nullptr) {
                continue;
            }

            if (_cameraProjectors.contains(cam->internalId())) {
                projectionModule = _cameraProjectors[cam->internalId()];
                solver->assignProjectorToFrame(projectionModule, imId);
            } else {
                PinholdeCamProjModule* pcpm = new PinholdeCamProjModule(cam);
                projectionModule = pcpm;
                bool ok = pcpm->init(solver, problem);

                if (!ok) {
                    return false;
                }

                _cameraProjectors[cam->internalId()] = projectionModule;
                solver->addProjector(projectionModule);
                solver->assignProjectorToFrame(projectionModule, imId);
            }
        }

        if (projectionModule == nullptr) {
            continue;
        }

        //GCPs
        for (qint64 imlmId : im->listTypedSubDataBlocks(ImageLandmark::ImageLandmarkClassName)) {

            ImageLandmark* iml = im->getImageLandmark(imlmId);

            if (iml == nullptr) {
                continue;
            }

            qint64 lmId = iml->attachedLandmarkid();

            if (!selection.pts.contains(lmId)) {
                continue;
            }

            ModularSBASolver::LandmarkNode* lmNode = solver->getNodeForLandmark(lmId, true);

            if (lmNode == nullptr) {
                continue;
            }

            Eigen::Vector2d ptPos;
            ptPos.x() = iml->x().value();
            ptPos.y() = iml->y().value();

            Eigen::Matrix2d info = Eigen::Matrix2d::Identity();
            info(0,0) = (iml->x().isUncertain()) ? 1./iml->x().stddev() : 1;
            info(1,1) = (iml->y().isUncertain()) ? 1./iml->y().stddev() : 1;

            projectionModule->addProjectionCostFunction(lmNode->pos.data(),
                                                        imNode->rAxis.data(),
                                                        imNode->t.data(),
                                                        ptPos,
                                                        info,
                                                        problem);
        }

        //stick to trajectory
        qint64 trajId = im->assignedTrajectory();
        std::optional<double> imTime = im->getImageTimestamp();

        ModularSBASolver::TrajectoryNode* tNode = solver->getNodeForTrajectory(trajId, false);
        ModularSBASolver::LeverArmNode* lvNode = solver->getNodeForLeverArm(QPair<qint64,qint64>(im->assignedCamera(),trajId), false);

        if (tNode != nullptr and lvNode != nullptr and imTime.has_value()) {

            double time = imTime.value();
            int nodeId = tNode->getNodeForTime(time);

            if (nodeId >= 0 and nodeId < tNode->nodes.size()-1) {
                ModularSBASolver::TrajectoryPoseNode& previous = tNode->nodes[nodeId];
                ModularSBASolver::TrajectoryPoseNode& next = tNode->nodes[nodeId+1];

                double pTime = previous.time;
                double nTime = next.time;

                double wP = (nTime - time)/(nTime-pTime);
                double wN = (time - pTime)/(nTime-pTime);

                if (std::abs(wP-1) < 1e-3 or !std::isfinite(wP)) {

                    ParametrizedLeverArmCostFunctor* cost =
                            new ParametrizedLeverArmCostFunctor();

                    using ceresFunc = ceres::AutoDiffCostFunction<ParametrizedLeverArmCostFunctor,6,3,3,3,3,3,3>;
                    ceresFunc* costFunc = new ceresFunc(cost);

                    problem.AddResidualBlock(costFunc, nullptr,
                                             lvNode->rAxis.data(), lvNode->t.data(),
                                             previous.rAxis.data(), previous.t.data(),
                                             imNode->rAxis.data(), imNode->t.data());

                } else if (std::abs(wN-1) < 1e-3) {

                    ParametrizedLeverArmCostFunctor* cost =
                            new ParametrizedLeverArmCostFunctor();

                    using ceresFunc = ceres::AutoDiffCostFunction<ParametrizedLeverArmCostFunctor,6,3,3,3,3,3,3>;
                    ceresFunc* costFunc = new ceresFunc(cost);

                    problem.AddResidualBlock(costFunc, nullptr,
                                             lvNode->rAxis.data(), lvNode->t.data(),
                                             next.rAxis.data(), next.t.data(),
                                             imNode->rAxis.data(), imNode->t.data());

                } else {

                    ParametrizedInterpolatedLeverArmCostFunctor* cost =
                            new ParametrizedInterpolatedLeverArmCostFunctor(wP, wN);

                    using ceresFunc = ceres::AutoDiffCostFunction<ParametrizedInterpolatedLeverArmCostFunctor,6,3,3,3,3,3,3,3,3>;
                    ceresFunc* costFunc = new ceresFunc(cost);

                    problem.AddResidualBlock(costFunc, nullptr,
                                             lvNode->rAxis.data(), lvNode->t.data(),
                                             previous.rAxis.data(), previous.t.data(),
                                             next.rAxis.data(), next.t.data(),
                                             imNode->rAxis.data(), imNode->t.data());
                }
            }

        }

    }

    return true;

}

bool ImageAlignementSBAModule::writeResults(ModularSBASolver* solver) {

    //Basic data structures are managed by the modular sba solver directly.
    return true;

}

bool ImageAlignementSBAModule::writeUncertainty(ModularSBASolver* solver) {

    //Basic data structures are managed by the modular sba solver directly.
    return true;
}

void ImageAlignementSBAModule::cleanup(ModularSBASolver* solver) {
    //Basic data structures are managed by the modular sba solver directly.
    Q_UNUSED(solver);
    return;
}



PinholdeCamProjModule::PinholdeCamProjModule(Camera* associatedCamera) :
    _associatedCamera(associatedCamera)
{

}

PinholdeCamProjModule::~PinholdeCamProjModule() {

}

bool PinholdeCamProjModule::addProjectionCostFunction(double* pointData,
                                                      double* poseOrientation,
                                                      double* posePosition,
                                                      Eigen::Vector2d const& ptProjPos,
                                                      Eigen::Matrix2d const& ptProjStiffness,
                                                      ceres::Problem & problem) {


    ParametrizedXYZ2UVCost* projCost = new ParametrizedXYZ2UVCost(ptProjPos, ptProjStiffness);

    problem.AddResidualBlock(new ceres::AutoDiffCostFunction<ParametrizedXYZ2UVCost, 2, 3, 3, 3, 1, 2, 3, 2, 2>(projCost), nullptr,
                              pointData,
                              poseOrientation,
                              posePosition,
                              &_fLen,
                              _principalPoint.data(),
                              _radialDistortion.data(),
                              _tangentialDistortion.data(),
                              _skewDistortion.data());

    /*if (_verbose) {

        std::array<double,3> res;

        (*projCost)(l_p.position.data(),
                    pose.rAxis.data(),
                    pose.t.data(),
                    &c_p.fLen,
                    c_p.principalPoint.data(),
                    c_p.radialDistortion.data(),
                    c_p.tangentialDistortion.data(),
                    c_p.skewDistortion.data(),
                    res.data());

        out << "Image GCP (img " << id << ") ";
        out << "lm[" << lmId << "] ";
        out << "initial residuals = [" << res[0] << " " << res[1] << "]\n";
    }*/

    return true;
}

bool PinholdeCamProjModule::addCrossProjectionCostFunction(double* pose1Orientation,
                                                           double* pose1Position,
                                                           Eigen::Vector2d const& ptProj1Pos,
                                                           Eigen::Matrix2d const& ptProj1Stiffness,
                                                           double* pose2Orientation,
                                                           double* pose2Position,
                                                           Eigen::Vector2d const& ptProj2Pos,
                                                           Eigen::Matrix2d const& ptProj2Stiffness,
                                                           ceres::Problem & problem) {

    //doing a uv2uv projection with distortion is non trivial. For this model of camera distortion it is better to use an intermediate landmark.
    return false;

}

bool PinholdeCamProjModule::init(ModularSBASolver* solver, ceres::Problem & problem) {

    Camera*& c = _associatedCamera;

    Eigen::Vector2d extend;
    extend.x() = c->imSize().width();
    extend.y() = c->imSize().height();

    if (c->optimizedOpticalCenterX().isSet() and
            c->optimizedOpticalCenterY().isSet()) {

        _principalPoint[0] = c->optimizedOpticalCenterX().value();
        _principalPoint[1] = c->optimizedOpticalCenterY().value();

    } else {

        _principalPoint[0] = c->opticalCenterX().value();
        _principalPoint[1] = c->opticalCenterY().value();
    }

    problem.AddParameterBlock(_principalPoint.data(), _principalPoint.size());

    if (c->optimizedFLen().isSet()) {
        _fLen = c->optimizedFLen().value();
    } else {
        _fLen = c->fLen().value();
    }

    problem.AddParameterBlock(&_fLen, 1);

    if (c->isFixed() or solver->getFixedParametersFlag()&FixedParameter::CameraInternal) {
        problem.SetParameterBlockConstant(&_fLen);
        problem.SetParameterBlockConstant(_principalPoint.data());
    }


    if (c->optimizedK1().isSet() and
            c->optimizedK2().isSet() and
            c->optimizedK3().isSet()) {

        _radialDistortion[0] = c->optimizedK1().value();
        _radialDistortion[1] = c->optimizedK2().value();
        _radialDistortion[2] = c->optimizedK3().value();

    } else {

        _radialDistortion[0] = c->k1().value();
        _radialDistortion[1] = c->k2().value();
        _radialDistortion[2] = c->k3().value();
    }

    problem.AddParameterBlock(_radialDistortion.data(), _radialDistortion.size());

    if (!c->useRadialDistortionModel()) {
        _radialDistortion[0] = 0;
        _radialDistortion[1] = 0;
        _radialDistortion[2] = 0;

        problem.SetParameterBlockConstant(_radialDistortion.data());
    }

    if (c->isFixed() or solver->getFixedParametersFlag()&FixedParameter::CameraInternal) {
        problem.SetParameterBlockConstant(_radialDistortion.data());
    }

    if (c->optimizedP1().isSet() and
                c->optimizedP2().isSet()) {

        _tangentialDistortion[0] = c->optimizedP1().value();
        _tangentialDistortion[1] = c->optimizedP2().value();

    } else {

        _tangentialDistortion[0] = c->p1().value();
        _tangentialDistortion[1] = c->p2().value();
    }

    problem.AddParameterBlock(_tangentialDistortion.data(), _tangentialDistortion.size());

    if (!c->useTangentialDistortionModel()) {
        _tangentialDistortion[0] = 0;
        _tangentialDistortion[1] = 0;

        problem.SetParameterBlockConstant(_tangentialDistortion.data());
    }

    if (c->isFixed() or solver->getFixedParametersFlag()&FixedParameter::CameraInternal) {
        problem.SetParameterBlockConstant(_tangentialDistortion.data());
    }

    if (c->optimizedB1().isSet() and
            c->optimizedB2().isSet()) {

        _skewDistortion[0] = c->optimizedB1().value();
        _skewDistortion[1] = c->optimizedB2().value();

    } else {

        _skewDistortion[0] = c->B1().value();
        _skewDistortion[1] = c->B2().value();
    }

    problem.AddParameterBlock(_skewDistortion.data(), _skewDistortion.size());

    if (!c->useSkewDistortionModel()) {
        _skewDistortion[0] = 0;
        _skewDistortion[1] = 0;

        problem.SetParameterBlockConstant(_skewDistortion.data());
    }

    if (c->isFixed() or solver->getFixedParametersFlag()&FixedParameter::CameraInternal) {

        problem.SetParameterBlockConstant(_skewDistortion.data());
    }

    return true;

}

bool PinholdeCamProjModule::writeResults(ModularSBASolver* solver) {



    Camera* cam = _associatedCamera;

    if (cam->isFixed()) {
        return true;
    }

    cam->clearOptimized();

    cam->setOptimizedFLen(static_cast<float>(_fLen));
    cam->setOptimizedOpticalCenterX(static_cast<float>(_principalPoint[0]));
    cam->setOptimizedOpticalCenterY(static_cast<float>(_principalPoint[1]));

    if (cam->useRadialDistortionModel()) {

        cam->setOptimizedK1(static_cast<float>(_radialDistortion[0]));
        cam->setOptimizedK2(static_cast<float>(_radialDistortion[1]));
        cam->setOptimizedK3(static_cast<float>(_radialDistortion[2]));
    }

    if (cam->useTangentialDistortionModel()) {

        cam->setOptimizedP1(static_cast<float>(_tangentialDistortion[0]));
        cam->setOptimizedP2(static_cast<float>(_tangentialDistortion[1]));
    }

    if (cam->useSkewDistortionModel()) {

        cam->setOptimizedB1(static_cast<float>(_skewDistortion[0]));
        cam->setOptimizedB2(static_cast<float>(_skewDistortion[1]));
    }

    return true;

}

bool PinholdeCamProjModule::writeUncertainty(ModularSBASolver* solver) {

    //TODO: get a way to write uncertainty.
    return true;
}

void PinholdeCamProjModule::cleanup(ModularSBASolver* solver) {
    Q_UNUSED(solver);
    return;
}

const char* SBASolverModulesInterface::AppInterfaceName = "StereoVisionApp::SBASolverModulesInterface";

SBASolverModulesInterface::SBASolverModulesInterface(QObject* parent) :
    QObject(parent)
{

}

} // namespace StereoVisionApp
