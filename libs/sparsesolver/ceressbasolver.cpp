#include "ceressbasolver.h"

#include <sbagraphreductor.h>

#include "datablocks/landmark.h"
#include "datablocks/image.h"
#include "datablocks/camera.h"
#include "datablocks/localcoordinatesystem.h"

#include "costfunctors/parametrizedxyz2uvcost.h"
#include "costfunctors/localpointalignementcost.h"

#include <ceres/cost_function.h>
#include <ceres/autodiff_cost_function.h>
#include <ceres/normal_prior.h>
#include <ceres/iteration_callback.h>

namespace StereoVisionApp {


class CeresSBASolverIterationCallback : public ceres::IterationCallback {
public:

    CeresSBASolverIterationCallback(CeresSBASolver* solver) :
        _solver(solver)
    {

    }

    virtual ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary) override {

        _solver->jumpStep();

        if (summary.step_is_valid) {
            return ceres::SOLVER_CONTINUE;
        }

        return ceres::SOLVER_ABORT;

    }

protected:

    CeresSBASolver* _solver;
};

CeresSBASolver::CeresSBASolver(Project* p, bool computeUncertainty, bool sparse, QObject* parent):
    SparseSolverBase(p, parent),
    _sparse(sparse),
    _compute_marginals(computeUncertainty)
{

}

int CeresSBASolver::uncertaintySteps() const {
    return (_compute_marginals) ? 1 : 0;
}

bool CeresSBASolver::hasUncertaintyStep() const {
    return _compute_marginals;
}

bool CeresSBASolver::init() {

    if (_currentProject == nullptr) {
        return false;
    }

    bool s = true;

    SBAGraphReductor selector(3,2,true,true);

    SBAGraphReductor::elementsSet selection = selector(_currentProject, false);

    if (selection.imgs.isEmpty() or selection.pts.isEmpty()) {
        return false;
    }

    // add points
    _landmarksParameters.reserve(selection.pts.size()); //reserve memory to avoid having stuff moved around.

    for (qint64 id : selection.pts) {
        Landmark* lm = qobject_cast<Landmark*>(_currentProject->getById(id));

        if (lm != nullptr) {

            _landmarksParameters.emplace_back();
            LandmarkPos& pos = _landmarksParameters.back();

            _landmarkParametersIndex.insert(id, _landmarksParameters.size()-1);

            pos.position[0] = lm->optPos().value(0);
            pos.position[1] = lm->optPos().value(1);
            pos.position[2] = lm->optPos().value(2);

            _problem.AddParameterBlock(pos.position.data(), pos.position.size());

            if (lm->xCoord().isSet() and lm->yCoord().isSet() and lm->zCoord().isSet()) {

                ceres::Vector m;
                m.resize(3);

                m[0] = lm->xCoord().value();
                m[1] = lm->yCoord().value();
                m[2] = lm->zCoord().value();

                Eigen::Matrix3d info = Eigen::Matrix3d::Identity();

                if (lm->xCoord().isUncertain()) {
                    info(0,0) = 1./(lm->xCoord().stddev()*lm->xCoord().stddev());
                } else {
                    info(0,0) = 1e6;
                }

                if (lm->yCoord().isUncertain()) {
                    info(1,1) = 1./(lm->yCoord().stddev()*lm->yCoord().stddev());
                } else {
                    info(1,1) = 1e6;
                }

                if (lm->zCoord().isUncertain()) {
                    info(2,2) = 1./(lm->zCoord().stddev()*lm->zCoord().stddev());
                } else {
                    info(2,2) = 1e6;
                }

                ceres::Matrix A = info;
                A.setConstant(3,3,0);

                A(0,0) = std::sqrt(info(0,0));
                A(1,1) = std::sqrt(info(0,0));
                A(2,2) = std::sqrt(info(0,0));

                ceres::NormalPrior* normalPrior = new ceres::NormalPrior(A, m);

                _problem.AddResidualBlock(normalPrior, nullptr, pos.position.data());

            }
        }
    }

    //add images
    _frameParameters.reserve(selection.imgs.size());
    _camParameters.reserve(_currentProject->countTypeInstances(Camera::staticMetaObject.className()));

    for (qint64 id : selection.imgs) {
        Image* im = qobject_cast<Image*>(_currentProject->getById(id));

        if (im != nullptr) {

            qint64 cam_id = setupCameraVertexForImage(im);

            if (cam_id < 0) {
                continue;
            }

            _frameParameters.emplace_back();
            FramePoseParameters& pose = _frameParameters.back();

            _frameParametersIndex.insert(id, _frameParameters.size()-1);

            pose.rAxis[0] = im->optRot().value(0);
            pose.rAxis[1] = im->optRot().value(1);
            pose.rAxis[2] = im->optRot().value(2);

            pose.t[0] = im->optPos().value(0);
            pose.t[1] = im->optPos().value(1);
            pose.t[2] = im->optPos().value(2);

            _problem.AddParameterBlock(pose.rAxis.data(), pose.rAxis.size());
            _problem.AddParameterBlock(pose.t.data(), pose.t.size());

            if (getFixedParametersFlag()&FixedParameter::CameraExternal) {
                _problem.SetParameterBlockConstant(pose.rAxis.data());
                _problem.SetParameterBlockConstant(pose.t.data());
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

                pose.rAxis = raxis_prior;
                pose.t = t_prior;

                _problem.SetParameterBlockConstant(pose.rAxis.data());
                _problem.SetParameterBlockConstant(pose.t.data());

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
                _problem.AddResidualBlock(tPrior, nullptr, pose.t.data());

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
                _problem.AddResidualBlock(rAxisPrior, nullptr, pose.rAxis.data());

            }

            for (qint64 lmId : im->listTypedSubDataBlocks(ImageLandmark::ImageLandmarkClassName)) {
                ImageLandmark* iml = im->getImageLandmark(lmId);

                if (iml != nullptr) {

                    qint64 lmId = iml->attachedLandmarkid();

                    if (!_landmarkParametersIndex.contains(lmId) or _camParametersIndex.contains(cam_id)) {
                        continue;
                    }

                    LandmarkPos& l_p = _landmarksParameters[_landmarkParametersIndex[lmId]];
                    CameraIntrinsicParameters& c_p = _camParameters[_camParametersIndex[cam_id]];

                    Eigen::Vector2d ptPos;
                    ptPos.x() = iml->x().value();
                    ptPos.y() = iml->y().value();

                    Eigen::Matrix2d info = Eigen::Matrix2d::Identity();
                    info(0,0) = (iml->x().isUncertain()) ? 1./iml->x().stddev() : 1;
                    info(1,1) = (iml->y().isUncertain()) ? 1./iml->y().stddev() : 1;

                    ParametrizedXYZ2UVCost* projCost = new ParametrizedXYZ2UVCost(ptPos, info);

                    _problem.AddResidualBlock(new ceres::AutoDiffCostFunction<ParametrizedXYZ2UVCost, 2, 3, 3, 3, 1, 2, 3, 2, 2>(projCost), nullptr,
                                              l_p.position.data(),
                                              pose.rAxis.data(),
                                              pose.t.data(),
                                              &c_p.fLen,
                                              c_p.principalPoint.data(),
                                              c_p.radialDistortion.data(),
                                              c_p.tangentialDistortion.data(),
                                              c_p.skewDistortion.data());
                }
            }
        }
    }

    // Local coordinate systems
    _localCoordinatesParameters.reserve(selection.localcoordsysts.size());

    for (qint64 id : selection.localcoordsysts) {
        LocalCoordinateSystem* lcs = _currentProject->getDataBlock<LocalCoordinateSystem>(id);

        if (lcs != nullptr) {

            _localCoordinatesParameters.emplace_back();
            FramePoseParameters& pose = _localCoordinatesParameters.back();

            _localCoordinatesParametersIndex.insert(id, _localCoordinatesParameters.size()-1);

            pose.rAxis[0] = M_PI;
            pose.rAxis[1] = 0;
            pose.rAxis[2] = 0;

            pose.t[0] = 0;
            pose.t[1] = 0;
            pose.t[2] = 1;

            if (lcs->optPos().isSet() and lcs->optRot().isSet()) {

                pose.rAxis[0] = lcs->optRot().value(0);
                pose.rAxis[1] = lcs->optRot().value(1);
                pose.rAxis[2] = lcs->optRot().value(2);

                pose.t[0] = lcs->optPos().value(0);
                pose.t[1] = lcs->optPos().value(1);
                pose.t[2] = lcs->optPos().value(2);
            }

            _problem.AddParameterBlock(pose.rAxis.data(), pose.rAxis.size());
            _problem.AddParameterBlock(pose.t.data(), pose.t.size());

            if (lcs->isFixed()) {
                _problem.SetParameterBlockConstant(pose.rAxis.data());
                _problem.SetParameterBlockConstant(pose.t.data());
            }

            for (qint64 lmId : lcs->listTypedSubDataBlocks(LandmarkLocalCoordinates::staticMetaObject.className())) {
                LandmarkLocalCoordinates* lmlc = lcs->getLandmarkLocalCoordinates(lmId);

                if (lmlc == nullptr) {
                    continue;
                }

                if (!lmlc->xCoord().isSet() or !lmlc->yCoord().isSet() or !lmlc->zCoord().isSet()) {
                    continue;
                }

                qint64 lm_id = lmlc->attachedLandmarkid();

                if (!_landmarkParametersIndex.contains(lm_id)) {
                    continue;
                }

                LandmarkPos& l_p = _landmarksParameters[_landmarkParametersIndex[lm_id]];

                Eigen::Vector3d localPos;
                localPos.x() = lmlc->xCoord().value();
                localPos.y() = lmlc->yCoord().value();
                localPos.z() = lmlc->zCoord().value();

                Eigen::Matrix3d info = Eigen::Matrix3d::Identity();

                LocalPointAlignementCost* alignementCost = new LocalPointAlignementCost(localPos, info);

                _problem.AddResidualBlock(new ceres::AutoDiffCostFunction<LocalPointAlignementCost, 3, 3, 3, 3>(alignementCost), nullptr,
                                          l_p.position.data(),
                                          pose.rAxis.data(),
                                          pose.t.data());
            }
        }
    }

    _not_first_step = false;
    return true;
}
bool CeresSBASolver::opt_step() {

    if (_not_first_step) {
        return true; //optimization already converged
    }

    ceres::Solver::Options options;
    options.linear_solver_type = (_sparse) ? ceres::SPARSE_SCHUR : ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    CeresSBASolverIterationCallback* iterationCallBack = new CeresSBASolverIterationCallback(this);

    options.max_num_iterations = optimizationSteps();

    options.callbacks = {iterationCallBack};

    ceres::Solver::Summary summary;

    ceres::Solve(options, &_problem, &summary);

    _not_first_step = true;

    return true; //TODO: implement
}
bool CeresSBASolver::std_step() {
    return true;
}
bool CeresSBASolver::writeResults() {

    if (_currentProject == nullptr) {
        return false;
    }

    for (qint64 id : _landmarkParametersIndex.keys()) {

        Landmark* lm = qobject_cast<Landmark*>(_currentProject->getById(id));

        if (lm->isFixed()) {
            continue;
        }

        LandmarkPos& lm_p = _landmarksParameters[_landmarkParametersIndex[id]];

        floatParameterGroup<3> pos;
        pos.value(0) = static_cast<float>(lm_p.position[0]);
        pos.value(1) = static_cast<float>(lm_p.position[1]);
        pos.value(2) = static_cast<float>(lm_p.position[2]);
        pos.setIsSet();
        lm->setOptPos(pos);

    }

    for (qint64 id : _frameParametersIndex.keys()) {

        Image* im = qobject_cast<Image*>(_currentProject->getById(id));

        if (im->isFixed()) {
            continue;
        }

        FramePoseParameters& im_p = _frameParameters[_frameParametersIndex[id]];

        floatParameterGroup<3> pos;
        pos.value(0) = static_cast<float>(im_p.t[0]);
        pos.value(1) = static_cast<float>(im_p.t[1]);
        pos.value(2) = static_cast<float>(im_p.t[2]);
        pos.setIsSet();
        im->setOptPos(pos);

        floatParameterGroup<3> rot;
        rot.value(0) = static_cast<float>(im_p.rAxis[0]);
        rot.value(1) = static_cast<float>(im_p.rAxis[0]);
        rot.value(2) = static_cast<float>(im_p.rAxis[0]);
        rot.setIsSet();
        im->setOptRot(rot);

    }

    for (qint64 id : _camParametersIndex.keys()) {

        Camera* cam = qobject_cast<Camera*>(_currentProject->getById(id));

        if (cam->isFixed()) {
            continue;
        }

        cam->clearOptimized();

        CameraIntrinsicParameters& cam_p = _camParameters[_camParametersIndex[id]];

        cam->setOptimizedFLen(static_cast<float>(cam_p.fLen));
        cam->setOptimizedOpticalCenterX(static_cast<float>(cam_p.principalPoint[0]));
        cam->setOptimizedOpticalCenterY(static_cast<float>(cam_p.principalPoint[1]));

        if (cam->useRadialDistortionModel()) {

            cam->setOptimizedK1(static_cast<float>(cam_p.radialDistortion[0]));
            cam->setOptimizedK2(static_cast<float>(cam_p.radialDistortion[1]));
            cam->setOptimizedK3(static_cast<float>(cam_p.radialDistortion[2]));
        }

        if (cam->useTangentialDistortionModel()) {

            cam->setOptimizedP1(static_cast<float>(cam_p.tangentialDistortion[0]));
            cam->setOptimizedP2(static_cast<float>(cam_p.tangentialDistortion[1]));
        }

        if (cam->useSkewDistortionModel()) {

            cam->setOptimizedB1(static_cast<float>(cam_p.skewDistortion[0]));
            cam->setOptimizedB2(static_cast<float>(cam_p.skewDistortion[1]));
        }

    }

    for (qint64 id : _localCoordinatesParametersIndex.keys()) {

        LocalCoordinateSystem* lcs = _currentProject->getDataBlock<LocalCoordinateSystem>(id);

        if (lcs->isFixed()) {
            continue;
        }

        FramePoseParameters& lcr_p = _localCoordinatesParameters[_localCoordinatesParametersIndex[id]];

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
bool CeresSBASolver::writeUncertainty() {
    return true; //TODO: implement
}

void CeresSBASolver::cleanup() {

    _landmarksParameters.clear();
    _landmarkParametersIndex.clear();

    _camParameters.clear();
    _camParametersIndex.clear();

    _frameParameters.clear();
    _frameParametersIndex.clear();

    _localCoordinatesParameters.clear();
    _localCoordinatesParametersIndex.clear();

    _problem = ceres::Problem(); //reset problem
}

bool CeresSBASolver::splitOptSteps() const {
    return true;
}


qint64 CeresSBASolver::setupCameraVertexForImage(Image* im) {

    if (im == nullptr) {
        return -1;
    }

    qint64 cam_id = im->assignedCamera();
    Camera* c = qobject_cast<Camera*>(_currentProject->getById(cam_id));

    if (c == nullptr) {
        return -1;
    }

    if (_camParametersIndex.contains(cam_id)) {
        return cam_id;
    }

    _camParameters.emplace_back();
    _camParametersIndex.insert(cam_id, _camParameters.size());

    CameraIntrinsicParameters& camParams = _camParameters.back();

    Eigen::Vector2d extend;
    extend.x() = c->imSize().width();
    extend.y() = c->imSize().height();

    if (c->optimizedOpticalCenterX().isSet() and
            c->optimizedOpticalCenterY().isSet()) {

        camParams.principalPoint[0] = c->optimizedOpticalCenterX().value();
        camParams.principalPoint[1] = c->optimizedOpticalCenterY().value();

    } else {

        camParams.principalPoint[0] = c->opticalCenterX().value();
        camParams.principalPoint[1] = c->opticalCenterY().value();
    }

    _problem.AddParameterBlock(camParams.principalPoint.data(), camParams.principalPoint.size());

    if (c->optimizedFLen().isSet()) {
        camParams.fLen = c->optimizedFLen().value();
    } else {
        camParams.fLen = c->fLen().value();
    }

    _problem.AddParameterBlock(&camParams.fLen, 1);

    if (c->isFixed() or getFixedParametersFlag()&FixedParameter::CameraInternal) {
        _problem.SetParameterBlockConstant(&camParams.fLen);
        _problem.SetParameterBlockConstant(camParams.principalPoint.data());
    }


    if (c->optimizedK1().isSet() and
            c->optimizedK2().isSet() and
            c->optimizedK3().isSet()) {

        camParams.radialDistortion[0] = c->optimizedK1().value();
        camParams.radialDistortion[1] = c->optimizedK2().value();
        camParams.radialDistortion[2] = c->optimizedK3().value();

    } else {

        camParams.radialDistortion[0] = c->k1().value();
        camParams.radialDistortion[1] = c->k2().value();
        camParams.radialDistortion[2] = c->k3().value();
    }

    _problem.AddParameterBlock(camParams.radialDistortion.data(), camParams.radialDistortion.size());

    if (!c->useRadialDistortionModel()) {
        camParams.radialDistortion[0] = 0;
        camParams.radialDistortion[1] = 0;
        camParams.radialDistortion[2] = 0;

        _problem.SetParameterBlockConstant(camParams.radialDistortion.data());
    }

    if (c->isFixed() or getFixedParametersFlag()&FixedParameter::CameraInternal) {
        _problem.SetParameterBlockConstant(camParams.radialDistortion.data());
    }

    if (c->optimizedP1().isSet() and
                c->optimizedP2().isSet()) {

        camParams.tangentialDistortion[0] = c->optimizedP1().value();
        camParams.tangentialDistortion[1] = c->optimizedP2().value();

    } else {

        camParams.tangentialDistortion[0] = c->p1().value();
        camParams.tangentialDistortion[1] = c->p2().value();
    }

    _problem.AddParameterBlock(camParams.tangentialDistortion.data(), camParams.tangentialDistortion.size());

    if (!c->useTangentialDistortionModel()) {
        camParams.tangentialDistortion[0] = 0;
        camParams.tangentialDistortion[1] = 0;

        _problem.SetParameterBlockConstant(camParams.tangentialDistortion.data());
    }

    if (c->isFixed() or getFixedParametersFlag()&FixedParameter::CameraInternal) {
        _problem.SetParameterBlockConstant(camParams.tangentialDistortion.data());
    }

    if (c->optimizedB1().isSet() and
            c->optimizedB2().isSet()) {

        camParams.skewDistortion[0] = c->optimizedB1().value();
        camParams.skewDistortion[1] = c->optimizedB2().value();

    } else {

        camParams.skewDistortion[0] = c->B1().value();
        camParams.skewDistortion[1] = c->B2().value();
    }

    _problem.AddParameterBlock(camParams.skewDistortion.data(), camParams.skewDistortion.size());

    if (!c->useSkewDistortionModel()) {
        camParams.skewDistortion[0] = 0;
        camParams.skewDistortion[1] = 0;

        _problem.SetParameterBlockConstant(camParams.skewDistortion.data());
    }

    if (c->isFixed() or getFixedParametersFlag()&FixedParameter::CameraInternal) {

        _problem.SetParameterBlockConstant(camParams.skewDistortion.data());
    }

    return cam_id;

}

} // namespace StereoVisionApp
