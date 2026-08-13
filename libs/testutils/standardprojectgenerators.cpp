#include "standardprojectgenerators.h"

#include <random>

#include "../../libs/datablocks/project.h"
#include "../../libs/datablocks/image.h"
#include "../../libs/datablocks/camera.h"
#include "../../libs/datablocks/landmark.h"
#include "../../libs/datablocks/correspondencesset.h"

#include "./datablocks/generatedtrajectory.h"

namespace StereoVisionApp {

namespace StandardProjectGenerators
{



bool simplePnPGenerator(Project* p, int seed, int nImages, bool withCorrespondance, bool fixedPoints) {

    if (p == nullptr) {
        return false;
    }

    if (nImages <= 0) {
        return false;
    }

    Project & project = *p;

    qint64 imageId = project.createDataBlock(StereoVisionApp::Image::staticMetaObject.className());
    qint64 cameraId = project.createDataBlock(StereoVisionApp::Camera::staticMetaObject.className());
    qint64 correspSetId = project.createDataBlock(StereoVisionApp::CorrespondencesSet::staticMetaObject.className());

    StereoVisionApp::Image* img = project.getDataBlock<StereoVisionApp::Image>(imageId); //first image, with fixed position
    StereoVisionApp::Camera* cam = project.getDataBlock<StereoVisionApp::Camera>(cameraId);
    StereoVisionApp::CorrespondencesSet* correspSet = project.getDataBlock<StereoVisionApp::CorrespondencesSet>(correspSetId);

    if (img == nullptr) {
        return false;
    }
    if (cam == nullptr) {
        return false;
    }
    if (correspSet == nullptr) {
        return false;
    }

    img->assignCamera(cameraId);

    cam->setImHeight(480);
    cam->setImWidth(640);

    StereoVisionApp::floatParameter fLen(35);
    StereoVisionApp::floatParameter ppX(pFloatType(cam->imWidth())/2);
    StereoVisionApp::floatParameter ppY(pFloatType(cam->imHeight())/2);
    fLen.setUncertainty(0.1);
    ppX.setUncertainty(0.1);
    ppY.setUncertainty(0.1);

    StereoVisionApp::floatParameter zero(0);
    zero.setUncertainty(0.1);

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

    //cam->setFixed(true); //ensure camera is fixed

    StereoVision::Geometry::RigidBodyTransform<double> cam2world(Eigen::Vector3d(M_PI,0,0), Eigen::Vector3d(0,0,30)); //cam2world

    StereoVision::Geometry::RigidBodyTransform<double> world2cam = cam2world.inverse();

    img->setXCoord(cam2world.t.x());
    img->setYCoord(cam2world.t.y());
    img->setZCoord(cam2world.t.z());

    img->setXRot(cam2world.r.x());
    img->setYRot(cam2world.r.y());
    img->setZRot(cam2world.r.z());

    std::vector<StereoVisionApp::Image*> images(nImages);
    std::vector<StereoVision::Geometry::RigidBodyTransform<double>> imgs2world(nImages);
    images[0] = img;
    imgs2world[0] = cam2world;

    std::default_random_engine re(seed);

    for (int i = 1; i < nImages; i++) {

        std::uniform_real_distribution<double> rangeDist(-1,1);

        double az = M_PI*rangeDist(re);
        double lz = M_PI*rangeDist(re);

        qint64 imageId = project.createDataBlock(StereoVisionApp::Image::staticMetaObject.className());

        StereoVisionApp::Image* img = project.getDataBlock<StereoVisionApp::Image>(imageId);

        img->assignCamera(cameraId);

        StereoVision::Geometry::RigidBodyTransform<double> cam2world =
            StereoVision::Geometry::RigidBodyTransform<double>(Eigen::Vector3d(0,0,az), Eigen::Vector3d(0,0,0))*
            StereoVision::Geometry::RigidBodyTransform<double>(Eigen::Vector3d(0,lz,0), Eigen::Vector3d(0,0,0))*
            StereoVision::Geometry::RigidBodyTransform<double>(Eigen::Vector3d(M_PI,0,0), Eigen::Vector3d(0,0,30)); //cam2world

        img->setXCoord(cam2world.t.x());
        img->setYCoord(cam2world.t.y());
        img->setZCoord(cam2world.t.z());

        img->setXRot(cam2world.r.x());
        img->setYRot(cam2world.r.y());
        img->setZRot(cam2world.r.z());

        images[i] = img;
        imgs2world[i] = cam2world;

    }

    std::vector<Eigen::Vector3d> points =
        {
            Eigen::Vector3d{3,2,1},
            Eigen::Vector3d{-2,0,-3},
            Eigen::Vector3d{-1,-1,3},
            Eigen::Vector3d{1,0,-2},
            Eigen::Vector3d{-1,3,0}
        };

    for (Eigen::Vector3d const& point : points) {

        constexpr StereoVisionApp::Correspondences::Types UV = StereoVisionApp::Correspondences::UV;
        constexpr StereoVisionApp::Correspondences::Types LmPos = StereoVisionApp::Correspondences::PRIORID;
        constexpr StereoVisionApp::Correspondences::Types PointPos = StereoVisionApp::Correspondences::GEOXYZ;

        using UVCorresp = StereoVisionApp::Correspondences::Typed<UV>;
        using PointCorresp = StereoVisionApp::Correspondences::Typed<PointPos>;
        using LmCorresp = StereoVisionApp::Correspondences::Typed<LmPos>;

        if (withCorrespondance and fixedPoints) {

            for (int i = 0; i < images.size(); i++) {

                StereoVision::Geometry::RigidBodyTransform<double> world2cam = imgs2world[i].inverse();
                Eigen::Vector3d ptCam = world2cam*point;

                Eigen::Vector2d uv = ptCam.block<2,1>(0,0)/ptCam.z() * fLen.value();
                uv.x() += ppX.value();
                uv.y() += ppY.value();

                UVCorresp uvCorresp;
                uvCorresp.blockId = images[i]->internalId();
                uvCorresp.u = uv.x();
                uvCorresp.v = uv.y();
                uvCorresp.sigmaU = 1;
                uvCorresp.sigmaV = 1;

                PointCorresp pointCorresp;
                pointCorresp.crsInfos = PointCorresp::LocalFrameName;
                pointCorresp.x = point.x();
                pointCorresp.y = point.y();
                pointCorresp.z = point.z();

                correspSet->addCorrespondence({uvCorresp, pointCorresp});
            }

            continue;
        }

        qint64 lmId = project.createDataBlock(StereoVisionApp::Landmark::staticMetaObject.className());

        StereoVisionApp::Landmark* lm = project.getDataBlock<StereoVisionApp::Landmark>(lmId);

        if (lm == nullptr) {
            return false;
        }

        lm->setXCoord(point.x());
        lm->setYCoord(point.y());
        lm->setZCoord(point.z());

        lm->setFixed(true);

        if (withCorrespondance) {

            for (int i = 0; i < images.size(); i++) {

                StereoVision::Geometry::RigidBodyTransform<double> world2cam = imgs2world[i].inverse();
                Eigen::Vector3d ptCam = world2cam*point;

                Eigen::Vector2d uv = ptCam.block<2,1>(0,0)/ptCam.z() * fLen.value();
                uv.x() += ppX.value();
                uv.y() += ppY.value();

                UVCorresp uvCorresp;
                uvCorresp.blockId = images[i]->internalId();
                uvCorresp.u = uv.x();
                uvCorresp.v = uv.y();
                uvCorresp.sigmaU = 1;
                uvCorresp.sigmaV = 1;

                LmCorresp pointCorresp;
                pointCorresp.blockId = lmId;

                correspSet->addCorrespondence({uvCorresp, pointCorresp});
            }

        } else {
            for (int i = 0; i < images.size(); i++) {

                StereoVision::Geometry::RigidBodyTransform<double> world2cam = imgs2world[i].inverse();
                Eigen::Vector3d ptCam = world2cam*point;

                Eigen::Vector2d uv = ptCam.block<2,1>(0,0)/ptCam.z() * fLen.value();
                uv.x() += ppX.value();
                uv.y() += ppY.value();

                images[i]->addImageLandmark(QPointF(uv.x(), uv.y()), lmId);
            }
        }

    }

    return true;

}


bool circularTrajectoryEcef(Project* p,
                            int seed,
                            float duration,
                            float samplingDt,
                            float accDt,
                            float posDt,
                            std::string const& name) {

    if (p == nullptr) {
        return false;
    }

    Project & project = *p;

    StereoVision::Geometry::AffineTransform<double> ecef2Local = project.ecef2local();
    StereoVision::Geometry::AffineTransform<double> local2Ecef(ecef2Local.R.transpose(), -ecef2Local.R.transpose()*ecef2Local.t);
    const char* ecefCRS = "EPSG:4978";

    qint64 trajectoryId = project.createDataBlock(StereoVisionApp::Trajectory::staticMetaObject.className());

    StereoVisionApp::Trajectory* traj = project.getDataBlock<StereoVisionApp::Trajectory>(trajectoryId);

    if (traj == nullptr) {
        return false;
    }

    StereoVisionApp::GeneratedTrajectory* genTraj = qobject_cast<StereoVisionApp::GeneratedTrajectory*>(traj);

    if (genTraj == nullptr) {
        return false;
    }

    genTraj->setObjectName(QString::fromStdString(name));

    genTraj->setPositionEpsg(ecefCRS);

    using TrajGeneratorInfos = StereoVisionApp::GeneratedTrajectory::TrajGeneratorInfos;

    constexpr double t0 = 0;
    double tf = duration;
    double dt = tf-t0;

    double rotationRate = 2*M_PI/(tf-t0);
    constexpr double circleRadius = 1000;

    double dtPos = posDt;
    double dtAcc = accDt;

    struct PositionCalculator {
        //position local
        static Eigen::Vector3d pos(double t, double dt) {
            double x = circleRadius*std::cos(t/dt * 2*M_PI);
            double y = circleRadius*std::sin(t/dt * 2*M_PI);
            double z = 0;
            return Eigen::Vector3d(x,y,z);
        }
        //speed local
        static Eigen::Vector3d speed(double t, double dt) {
            double x = -circleRadius*std::sin(t/dt * 2*M_PI)/dt * 2*M_PI;
            double y = circleRadius*std::cos(t/dt * 2*M_PI)/dt * 2*M_PI;
            double z = 0;
            return Eigen::Vector3d(x,y,z);
        }
        //orientation local
        static Eigen::Vector3d orientation(double t, double rotationRate) {
            double rz = t*rotationRate - M_PI_2;
            return Eigen::Vector3d(0,0,rz);
        }

        static Eigen::Vector3d coriolisEcefModel(Eigen::Vector3d const& ECEFPos, Eigen::Vector3d const& ECEFSpeed) {
            return 2*Eigen::Vector3d(0, 0, StereoVisionApp::Geo::WGS84Ellipsoid::EarthRotationRate).cross(ECEFSpeed);
        }
    };

    Eigen::Vector3d earthRotation(0,0,StereoVisionApp::Geo::WGS84Ellipsoid::EarthRotationRate);
    Eigen::Vector3d earthRotationLocal = ecef2Local.R*earthRotation;

    genTraj->setInitialAndFinalTimes(t0,tf);

    genTraj->setAngularSpeedGenerator(TrajGeneratorInfos{[earthRotationLocal, rotationRate] (double t) {
                                                             double omega = rotationRate;
                                                             Eigen::Matrix3d Rlocal2Body = StereoVision::Geometry::rodriguezFormula<double>(-PositionCalculator::orientation(t, rotationRate));
                                                             Eigen::Vector3d ret = (Rlocal2Body*earthRotationLocal)+Eigen::Vector3d(0,0,omega);
                                                             return ret;
                                                         }, dtAcc}); //constant rotation rate
    genTraj->setAccelerationGenerator(TrajGeneratorInfos{[local2Ecef, dt, rotationRate] (double t) {
                                                             double acc = -circleRadius*rotationRate*rotationRate;
                                                             Eigen::Vector3d ecef = local2Ecef*PositionCalculator::pos(t, dt);
                                                             Eigen::Vector3d speed = local2Ecef.R*PositionCalculator::speed(t, dt);
                                                             std::array<double,3> gArray = StereoVisionApp::Geo::WGS84Ellipsoid::gravityEcefModel(ecef); //g keep its amplitude and points toward center of the earth
                                                             Eigen::Vector3d g(gArray[0],gArray[1],gArray[2]);
                                                             Eigen::Vector3d coriolis = PositionCalculator::coriolisEcefModel(ecef, speed);
                                                             Eigen::Matrix3d Rlocal2Body = StereoVision::Geometry::rodriguezFormula<double>(-PositionCalculator::orientation(t, rotationRate));

                                                             Eigen::Vector3d gLocal = local2Ecef.R.transpose()*g;
                                                             Eigen::Vector3d gBody = Rlocal2Body*gLocal;

                                                             Eigen::Vector3d coriolisLocal = local2Ecef.R.transpose()*coriolis;
                                                             Eigen::Vector3d coriolisBody = Rlocal2Body*coriolisLocal;

                                                             Eigen::Vector3d reorientedGravity = coriolisBody+gBody;

                                                             Eigen::Vector3d ret = Eigen::Vector3d(0,acc,0) +
                                                                                   reorientedGravity;
                                                             return ret;
                                                         }, dtAcc});

    genTraj->setPositionGenerator(TrajGeneratorInfos{[local2Ecef, dt] (double t) {
                                                         Eigen::Vector3d vec = local2Ecef*PositionCalculator::pos(t, dt);
                                                         return vec;
                                                     }, dtPos});

    constexpr StereoVisionApp::Geo::TopocentricConvention topocentricConvention = StereoVisionApp::Geo::ENU;

    genTraj->setOrientationGenerator(TrajGeneratorInfos{[local2Ecef, dt, rotationRate] (double t) {
                                                            Eigen::Vector3d ecef = local2Ecef*PositionCalculator::pos(t, dt);
                                                            Eigen::Matrix3d body2ecef = local2Ecef.R *
                                                                                        StereoVision::Geometry::rodriguezFormula<double>(PositionCalculator::orientation(t, rotationRate));
                                                            Eigen::Matrix3d topocentric2ecef = StereoVisionApp::Geo::localFrame2ECEFFromECEF(ecef, topocentricConvention);
                                                            return StereoVision::Geometry::inverseRodriguezFormula<double>(topocentric2ecef.transpose()*body2ecef); //body2topocentric
                                                        }, dtPos});
    traj->setOrientationAngleRepresentation(StereoVisionApp::Trajectory::AxisAngle);
    traj->setOrientationAngleUnits(StereoVisionApp::Trajectory::Radians);
    traj->setOrientationTopocentricConvention(topocentricConvention);

    traj->setPreIntegrationTime(samplingDt);
    traj->setGpsAccuracy(0.02);
    traj->setGyroAccuracy(0.1);
    traj->setAccAccuracy(0.5);

    return true;

}


} // namespace StandardProjectGenerators

} // namespace StereoVisionApp
