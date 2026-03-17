#include <iostream>
#include <cmath>
#include <vector>
#include <random>
#include <memory>

#include <StereoVision/geometry/rotations.h>

#include "../../libs/datablocks/project.h"
#include "../../libs/datablocks/landmark.h"
#include "../../libs/datablocks/image.h"
#include "../../libs/datablocks/camera.h"

#include "../../libs/sparsesolver/modularsbasolver.h"
#include "../../libs/sparsesolver/sbamodules/landmarkssbamodule.h"
#include "../../libs/sparsesolver/sbamodules/pinholecameraprojectormodule.h"
#include "../../libs/sparsesolver/sbamodules/imagealignementsbamodule.h"

using namespace StereoVisionApp;

using ComputeT = double;

struct ExperimentData{

    using FrameT = StereoVision::Geometry::RigidBodyTransform<ComputeT>;
    using PtT = Eigen::Matrix<ComputeT,3,1>;

    ComputeT fLen;
    std::vector<FrameT> cameras;
    std::vector<PtT> points;
};

ExperimentData generateExperiment(int nFrames, int nPoints, std::default_random_engine & re) {

    //we generate a sphere of radius 1, generate all the points inside this sphere
    //then we generate all the frame on the surface of a sphere of radius 2, pointing towards the center of the scene
    //finally, we apply a small perturbation to the position and orientation of the cameras. We assume an fLen of 800 px.

    using AngleVecT = Eigen::Matrix<ComputeT,3,1>;
    using PosVecT = Eigen::Matrix<ComputeT,3,1>;

    ExperimentData ret;
    ret.fLen = 800;

    std::uniform_real_distribution<ComputeT> ptDist(-1,1);

    ret.points.resize(nPoints);
    for (int i = 0; i < nPoints; i++) {

        ExperimentData::PtT pt(0,0,1);
        ComputeT norm;
        do {
            pt << ptDist(re), ptDist(re), ptDist(re);
            norm = pt.norm();
        } while (norm > 1); //rejection sampling

        ret.points[i] = pt;
    }

    std::uniform_real_distribution<ComputeT> projDist(0,1);
    ret.cameras.resize(nFrames);
    for (int i = 0; i < nFrames; i++) {

        ComputeT u1 = projDist(re);
        ComputeT u2 = projDist(re);

        PosVecT pos;

        ComputeT z = 2*u1-1;
        ComputeT r = sqrt(1-z*z);
        ComputeT phi = 2*M_PI*u2;
        ComputeT x = std::cos(phi)*r;
        ComputeT y = std::sin(phi)*r;

        pos <<  x,y,z;
        pos *= 2;

        AngleVecT orientation = -PosVecT(0,0,1).cross(pos.normalized());
        ComputeT sinTheta = orientation.norm();
        ComputeT theta = std::asin(sinTheta);

        if (pos.z() < 0) {
            theta += M_PI_2;
        }

        orientation = theta*orientation.normalized();

        ComputeT rotScale = 2*M_PI*projDist(re);

        ExperimentData::FrameT oriented2cam = ExperimentData::FrameT(rotScale*PosVecT(0,0,1), PosVecT::Zero());
        ExperimentData::FrameT world2cam = oriented2cam*ExperimentData::FrameT(orientation, PosVecT::Zero());

        ExperimentData::FrameT cam2world(-world2cam.r, -pos);

        ret.cameras[i] = cam2world;

    }

    return ret;
}

int main(int argc, char** argv) {

    google::InitGoogleLogging(argv[0]);

    std::default_random_engine re;
    re.seed(42); //fix seed

    constexpr int nFrames = 6;
    constexpr int nPoints = 15;
    constexpr int nGCP = 3; //minimum to set scene scale and orientation

    ExperimentData data = generateExperiment(nFrames, nPoints, re);

    ProjectFactory& pF = ProjectFactory::defaultProjectFactory();

    pF.addType(new StereoVisionApp::LandmarkFactory(&pF));
    pF.addType(new StereoVisionApp::ImageFactory(&pF));
    pF.addType(new StereoVisionApp::CameraFactory(&pF));

    std::unique_ptr<Project> pPtr(pF.createProject(nullptr)); //use unique ptr to ensure project is deleted at the end of test case

    Project& project = *pPtr;

    bool computeUncertainty = false;
    bool sparse = true;
    bool verbose = true;

    ModularSBASolver sbaSolver(&project, computeUncertainty, sparse, verbose);
    sbaSolver.setProperty(PinholeCamProjModule::UseSphericalCostSolverParamName, true); //use the spherical projector

    bool lmModuleAdded = sbaSolver.addModule(new LandmarksSBAModule());
    bool imModuleAdded = sbaSolver.addModule(new ImageAlignementSBAModule());

    if (!lmModuleAdded) {
        std::cerr << "Failed to add landmark module" << std::endl;
        return 1;
    }
    if (!imModuleAdded) {
        std::cerr << "Failed to add image module" << std::endl;
        return 1;
    }

    ComputeT sPt = 0.1;
    ComputeT sPtInitial = 0.3;
    std::uniform_real_distribution<ComputeT> ptInitialDeltaDist(-2*sPtInitial,2*sPtInitial);

    std::vector<Landmark*> lms(data.points.size());

    for (int i = 0; i < data.points.size(); i++) {
        int lmId = project.createDataBlock(Landmark::staticMetaObject.className());

        Landmark* lm = project.getDataBlock<Landmark>(lmId);

        lms[i] = lm;

        if (lm == nullptr) {
            continue;
        }

        ComputeT dxInitial = ptInitialDeltaDist(re);
        ComputeT dyInitial = ptInitialDeltaDist(re);
        ComputeT dzInitial = ptInitialDeltaDist(re);

        floatParameterGroup<3> optPos;
        optPos.setIsSet();
        optPos.value(0) = data.points[i].x() + dxInitial;
        optPos.value(1) = data.points[i].y() + dyInitial;
        optPos.value(2) = data.points[i].z() + dzInitial;

        //set a noisy initial solution
        lm->setOptPos(optPos);

        if (i < nGCP) {
            //set a correct prior
            lm->setXCoord(floatParameter(data.points[i].x(), sPt));
            lm->setYCoord(floatParameter(data.points[i].y(), sPt));
            lm->setZCoord(floatParameter(data.points[i].z(), sPt));
        }
    }

    qint64 cameraId = project.createDataBlock(Camera::staticMetaObject.className());
    Camera* cam = project.getDataBlock<Camera>(cameraId);

    cam->setImHeight(480);
    cam->setImWidth(640);

    StereoVisionApp::floatParameter zero(0);

    StereoVisionApp::floatParameter fLen(data.fLen);
    StereoVisionApp::floatParameter ppX(pFloatType(cam->imWidth())/2);
    StereoVisionApp::floatParameter ppY(pFloatType(cam->imHeight())/2);

    cam->setFLen(fLen);
    cam->setOpticalCenterX(ppX);
    cam->setOpticalCenterY(ppY);

    cam->setB1(zero);
    cam->setB2(zero);

    cam->setP1(zero);
    cam->setP2(zero);

    cam->setK1(zero);
    cam->setK2(zero);
    cam->setK3(zero);
    cam->setK4(zero);
    cam->setK5(zero);
    cam->setK6(zero);

    std::uniform_real_distribution<ComputeT> rotInitialDeltaDist(-2*sPtInitial,2*sPtInitial);

    std::vector<Image*> imgs(data.cameras.size());

    for (int i = 0; i < data.cameras.size(); i++) {
        int imId = project.createDataBlock(Image::staticMetaObject.className());

        Image* img = project.getDataBlock<Image>(imId);

        imgs[i] = img;

        if (img == nullptr) {
            continue;
        }

        img->assignCamera(cameraId);

        ComputeT dxInitial = ptInitialDeltaDist(re);
        ComputeT dyInitial = ptInitialDeltaDist(re);
        ComputeT dzInitial = ptInitialDeltaDist(re);

        ComputeT drxInitial = rotInitialDeltaDist(re);
        ComputeT dryInitial = rotInitialDeltaDist(re);
        ComputeT drzInitial = rotInitialDeltaDist(re);

        ExperimentData::FrameT const& cam2world = data.cameras[i];
        ExperimentData::FrameT world2cam = cam2world.inverse();

        floatParameterGroup<3> optPos;
        optPos.setIsSet();
        optPos.value(0) = cam2world.t.x() + dxInitial;
        optPos.value(1) = cam2world.t.y() + dyInitial;
        optPos.value(2) = cam2world.t.z() + dzInitial;

        floatParameterGroup<3> optRot;
        optRot.setIsSet();
        optRot.value(0) = cam2world.r.x() + drxInitial;
        optRot.value(1) = cam2world.r.y() + dryInitial;
        optRot.value(2) = cam2world.r.z() + drzInitial;

        //set a noisy initial solution
        img->setOptPos(optPos);
        img->setOptRot(optRot);

        for (int i = 0; i < data.points.size(); i++) {

            if (lms[i] == nullptr) {
                continue;
            }

            ExperimentData::PtT const& point = data.points[i];

            Eigen::Vector3d ptCam = world2cam*point;

            if (ptCam.z() < 0) {
                std::cerr << "Issue in randomly generated problem! 1 point is behind a camera!" << std::endl;
                return 1;
            }

            Eigen::Vector2d uv = ptCam.block<2,1>(0,0)/ptCam.z() * fLen.value();
            uv.x() += ppX.value();
            uv.y() += ppY.value();

            img->addImageLandmark(QPointF(uv.x(), uv.y()), lms[i]->internalId());
        }
    }

    bool initSuccess = sbaSolver.init();

    if (!initSuccess) {
        std::cerr << "Problem, cannot init the solver!" << std::endl;
        return 1;
    }

    bool opt_ok = sbaSolver.opt_step();

    if (!opt_ok) {
        std::cerr << "Optimization failed!" << std::endl;
        return 1;
    }

    return 0;

}
