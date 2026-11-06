#include <QtTest/QtTest>

#include <time.h>

#include "../../libs/datablocks/project.h"
#include "../../libs/datablocks/image.h"
#include "../../libs/datablocks/camera.h"
#include "../../libs/datablocks/landmark.h"
#include "../../libs/datablocks/correspondencesset.h"
#include "../../libs/datablocks/localcoordinatesystem.h"
#include "../../libs/datablocks/trajectory.h"
#include "../../libs/datablocks/mounting.h"

#include "../../libs/sparsesolver/modularsbasolver.h"
#include "../../libs/sparsesolver/sbamodules/landmarkssbamodule.h"
#include "../../libs/sparsesolver/sbamodules/pinholecameraprojectormodule.h"
#include "../../libs/sparsesolver/sbamodules/imagealignementsbamodule.h"
#include "../../libs/sparsesolver/sbamodules/correspondencessetsbamodule.h"
#include "../../libs/sparsesolver/sbamodules/localcoordinatesystemsbamodule.h"
#include "../../libs/sparsesolver/sbamodules/trajectorybasesbamodule.h"
#include "../../libs/sparsesolver/sbamodules/mountingssbamodule.h"

#include "../../libs/testutils/datablocks/generatedtrajectory.h"
#include "../../libs/testutils/waypointssimulatedtrajectorysmoother.h"
#include "../../libs/testutils/eceftrajectoryinertialinstrumentssimulator.h"

#include "../../libs/geo/wgs84.h"

#include <QDebug>


class TestSBASolver : public QObject
{
    Q_OBJECT
public:

private Q_SLOTS:

    void initTestCase();

    void simplePnP_data();
    void simplePnP();

    void imagePair();

    void image2RigidBody();
    void rigidBody2RigidBody();

    void basicTrajectory_data();
    void basicTrajectory();
    void movingBody2MovingBody();

    void testFixedWithEarthGravity();
    void testWithEarthGravity();
    /*!
     * \brief testWaypointTrajectoryArpette simulated flights with waypoints above val d'arpette
     */
    void testWaypointTrajectoryArpette();

    void testCameras2Trajectory();

protected:

    void genericPnPTest(bool withCorrespondance, bool fixedPoints);
    void configureStandardTrajectory(StereoVisionApp::Trajectory* traj, QString const& sbet, QString const& ins);

    std::default_random_engine _re;
};

void TestSBASolver::genericPnPTest(bool withCorrespondance, bool fixedPoints) {

    StereoVisionApp::ProjectFactory& pF = StereoVisionApp::ProjectFactory::defaultProjectFactory();

    std::unique_ptr<StereoVisionApp::Project> pPtr(pF.createProject(this)); //use unique ptr to ensure project is deleted at the end of test case

    QVERIFY(pPtr != nullptr);

    StereoVisionApp::Project& project = *pPtr;

    qint64 imageId = project.createDataBlock(StereoVisionApp::Image::staticMetaObject.className());
    qint64 cameraId = project.createDataBlock(StereoVisionApp::Camera::staticMetaObject.className());
    qint64 correspSetId = project.createDataBlock(StereoVisionApp::CorrespondencesSet::staticMetaObject.className());

    StereoVisionApp::Image* img = project.getDataBlock<StereoVisionApp::Image>(imageId);
    StereoVisionApp::Camera* cam = project.getDataBlock<StereoVisionApp::Camera>(cameraId);
    StereoVisionApp::CorrespondencesSet* correspSet = project.getDataBlock<StereoVisionApp::CorrespondencesSet>(correspSetId);

    QVERIFY(img != nullptr);
    QVERIFY(cam != nullptr);
    QVERIFY(correspSet != nullptr);

    img->assignCamera(cameraId);

    cam->setImHeight(480);
    cam->setImWidth(640);

    StereoVisionApp::floatParameter fLen(35);
    StereoVisionApp::floatParameter ppX(pFloatType(cam->imWidth())/2);
    StereoVisionApp::floatParameter ppY(pFloatType(cam->imHeight())/2);

    StereoVisionApp::floatParameter zero(0);

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

    StereoVision::Geometry::RigidBodyTransform<double> cam2world(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()); //cam2world

    StereoVision::Geometry::RigidBodyTransform<double> world2cam = cam2world.inverse();

    img->setXCoord(cam2world.t.x());
    img->setYCoord(cam2world.t.y());
    img->setZCoord(cam2world.t.z());

    img->setXRot(cam2world.r.x());
    img->setYRot(cam2world.r.y());
    img->setZRot(cam2world.r.z());

    std::vector<Eigen::Vector3d> points =
        {
            Eigen::Vector3d{42,69,33},
            Eigen::Vector3d{33,27,50},
            Eigen::Vector3d{-6,-99,70},
            Eigen::Vector3d{-49,-28,12},
            Eigen::Vector3d{-69,-42,33}
        };

    for (Eigen::Vector3d const& point : points) {

        constexpr StereoVisionApp::Correspondences::Types UV = StereoVisionApp::Correspondences::UV;
        constexpr StereoVisionApp::Correspondences::Types LmPos = StereoVisionApp::Correspondences::PRIORID;
        constexpr StereoVisionApp::Correspondences::Types PointPos = StereoVisionApp::Correspondences::GEOXYZ;

        using UVCorresp = StereoVisionApp::Correspondences::Typed<UV>;
        using PointCorresp = StereoVisionApp::Correspondences::Typed<PointPos>;
        using LmCorresp = StereoVisionApp::Correspondences::Typed<LmPos>;

        Eigen::Vector3d ptCam = world2cam*point;

        Eigen::Vector2d uv = ptCam.block<2,1>(0,0)/ptCam.z() * fLen.value();
        uv.x() += ppX.value();
        uv.y() += ppY.value();

        if (withCorrespondance and fixedPoints) {


            UVCorresp uvCorresp;
            uvCorresp.blockId = imageId;
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

            continue;
        }

        qint64 lmId = project.createDataBlock(StereoVisionApp::Landmark::staticMetaObject.className());

        StereoVisionApp::Landmark* lm = project.getDataBlock<StereoVisionApp::Landmark>(lmId);

        QVERIFY(lm != nullptr);

        lm->setXCoord(point.x());
        lm->setYCoord(point.y());
        lm->setZCoord(point.z());

        lm->setFixed(true);

        if (withCorrespondance) {

            UVCorresp uvCorresp;
            uvCorresp.blockId = imageId;
            uvCorresp.u = uv.x();
            uvCorresp.v = uv.y();
            uvCorresp.sigmaU = 1;
            uvCorresp.sigmaV = 1;

            LmCorresp pointCorresp;
            pointCorresp.blockId = lmId;

            correspSet->addCorrespondence({uvCorresp, pointCorresp});

        } else {
            img->addImageLandmark(QPointF(uv.x(), uv.y()), lmId);
        }

    }


    StereoVisionApp::ModularSBASolver sbaSolver(&project);
    sbaSolver.setSilent(true);

    bool lmModuleAdded = sbaSolver.addModule(new StereoVisionApp::LandmarksSBAModule());
    bool imModuleAdded = sbaSolver.addModule(new StereoVisionApp::ImageAlignementSBAModule());
    bool correspModuleAdded = sbaSolver.addModule(new StereoVisionApp::CorrespondencesSetSBAModule());

    QVERIFY(lmModuleAdded);
    QVERIFY(imModuleAdded);
    QVERIFY(correspModuleAdded);

    bool initSuccess = sbaSolver.init();

    QVERIFY(initSuccess);

    QVERIFY(sbaSolver.itemIsObservable(imageId));

    ceres::Problem* problem = sbaSolver.ceresProblem();

    QVERIFY(problem != nullptr);

    //check something was added in the factor graph
    QVERIFY(problem->NumResidualBlocks() > 0);

    double cost;

    ceres::Problem::EvaluateOptions options;

    bool evaluateOk = problem->Evaluate(options, &cost, nullptr, nullptr, nullptr);

    QVERIFY(evaluateOk);

    QVERIFY(std::abs(cost) < 1e-6); //the problem should be initialized with a perfect solution

}

void TestSBASolver::configureStandardTrajectory(StereoVisionApp::Trajectory* traj, QString const& sbet, QString const& ins) {

    traj->setPositionFile(sbet);
    traj->setPositionEpsg("");
    traj->setPositionTimeDelta(0);
    traj->setPositionTimeScale(1);
    traj->setPositionColumn(StereoVisionApp::Trajectory::Axis::T,0);
    traj->setPositionColumn(StereoVisionApp::Trajectory::Axis::X,1);
    traj->setPositionColumn(StereoVisionApp::Trajectory::Axis::Y,2);
    traj->setPositionColumn(StereoVisionApp::Trajectory::Axis::Z,3);

    traj->setOrientationFile(sbet);
    traj->setOrientationTopocentricConvention(StereoVisionApp::Trajectory::TopocentricConvention::NED);
    traj->setOrientationAngleRepresentation(StereoVisionApp::Trajectory::AngleRepresentation::AxisAngle);
    traj->setOrientationAngleUnits(StereoVisionApp::Trajectory::AngleUnits::Radians);
    traj->setOrientationTimeDelta(0);
    traj->setOrientationTimeScale(1);
    traj->setOrientationColumn(StereoVisionApp::Trajectory::Axis::T,0);
    traj->setOrientationColumn(StereoVisionApp::Trajectory::Axis::X,4);
    traj->setOrientationColumn(StereoVisionApp::Trajectory::Axis::Y,5);
    traj->setOrientationColumn(StereoVisionApp::Trajectory::Axis::Z,6);
    traj->setOrientationSign(StereoVisionApp::Trajectory::Axis::X, 1);
    traj->setOrientationSign(StereoVisionApp::Trajectory::Axis::Y, 1);
    traj->setOrientationSign(StereoVisionApp::Trajectory::Axis::Z, 1);
    traj->setOrientationSign(StereoVisionApp::Trajectory::Axis::W, 1);

    traj->setAccelerometerFile(ins);
    traj->setAccelerometerId(0);
    traj->setAccelerometerMounting(StereoVision::Geometry::AffineTransform<double>(Eigen::Matrix3d::Identity(),
                                                                                   Eigen::Vector3d::Zero()));
    traj->setAccelerometerTimeDelta(0);
    traj->setAccelerometerTimeScale(1);
    traj->setEstAccelerometerScale(false);
    traj->setEstAccelerometerBias(false);
    traj->setAccelerometerColumn(StereoVisionApp::Trajectory::Axis::T,0);
    traj->setAccelerometerColumn(StereoVisionApp::Trajectory::Axis::X,1);
    traj->setAccelerometerColumn(StereoVisionApp::Trajectory::Axis::Y,2);
    traj->setAccelerometerColumn(StereoVisionApp::Trajectory::Axis::Z,3);

    traj->setGyroFile(ins);
    traj->setGyroAngleRepresentation(StereoVisionApp::Trajectory::AngleRepresentation::AxisAngle);
    traj->setGyroAngleUnits(StereoVisionApp::Trajectory::AngleUnits::Radians);
    traj->setGyroTimeDelta(0);
    traj->setGyroTimeScale(1);
    traj->setEstGyroScale(false);
    traj->setEstGyroBias(false);
    traj->setGyroColumn(StereoVisionApp::Trajectory::Axis::T,0);
    traj->setGyroColumn(StereoVisionApp::Trajectory::Axis::X,4);
    traj->setGyroColumn(StereoVisionApp::Trajectory::Axis::Y,5);
    traj->setGyroColumn(StereoVisionApp::Trajectory::Axis::Z,6);
    traj->setGyroColumn(StereoVisionApp::Trajectory::Axis::W,-1);
    traj->setGyroSign(StereoVisionApp::Trajectory::Axis::X, 1);
    traj->setGyroSign(StereoVisionApp::Trajectory::Axis::Y, 1);
    traj->setGyroSign(StereoVisionApp::Trajectory::Axis::Z, 1);
    traj->setGyroSign(StereoVisionApp::Trajectory::Axis::W, 1);

    traj->setPreIntegrationTime(0.1);
    traj->setGpsAccuracy(0.02);
    traj->setGyroAccuracy(0.1);
    traj->setAccAccuracy(0.5);
}

void TestSBASolver::initTestCase() {

    constexpr bool seed = 42;

    srand(seed);

    //configure libraries
    google::InitGoogleLogging("TestSBASolver");

    StereoVisionApp::ProjectFactory& pF = StereoVisionApp::ProjectFactory::defaultProjectFactory();

    pF.addType(new StereoVisionApp::LandmarkFactory(this));
    pF.addType(new StereoVisionApp::ImageFactory(this));
    pF.addType(new StereoVisionApp::CameraFactory(this));
    pF.addType(new StereoVisionApp::CorrespondencesSetFactory(this));
    pF.addType(new StereoVisionApp::LocalCoordinateSystemFactory(this));
    pF.addType(new StereoVisionApp::GeneratedTrajectoryFactory(this)); //ensure the trajectores can be configured with generators
    pF.addType(new StereoVisionApp::MountingFactory(this));

    _re.seed(seed);
}

void TestSBASolver::simplePnP_data() {

    QTest::addColumn<bool>("withCorrespondance");
    QTest::addColumn<bool>("fixedPoints");

    QTest::newRow("image Landmarks") << false << false;
    QTest::newRow("correspondance to landmarks") << true << false;
    QTest::newRow("correspondance to coordinates") << true << true;

}

void TestSBASolver::simplePnP() {
        QFETCH(bool, withCorrespondance);
        QFETCH(bool, fixedPoints);

        genericPnPTest(withCorrespondance, fixedPoints);
}


void TestSBASolver::imagePair() {

    StereoVisionApp::ProjectFactory& pF = StereoVisionApp::ProjectFactory::defaultProjectFactory();

    std::unique_ptr<StereoVisionApp::Project> pPtr(pF.createProject(this)); //use unique ptr to ensure project is deleted at the end of test case

    QVERIFY(pPtr != nullptr);

    StereoVisionApp::Project& project = *pPtr;

    qint64 image1Id = project.createDataBlock(StereoVisionApp::Image::staticMetaObject.className());
    qint64 image2Id = project.createDataBlock(StereoVisionApp::Image::staticMetaObject.className());
    qint64 cameraId = project.createDataBlock(StereoVisionApp::Camera::staticMetaObject.className());
    qint64 correspSetId = project.createDataBlock(StereoVisionApp::CorrespondencesSet::staticMetaObject.className());

    StereoVisionApp::Image* img1 = project.getDataBlock<StereoVisionApp::Image>(image1Id);
    StereoVisionApp::Image* img2 = project.getDataBlock<StereoVisionApp::Image>(image2Id);
    StereoVisionApp::Camera* cam = project.getDataBlock<StereoVisionApp::Camera>(cameraId);
    StereoVisionApp::CorrespondencesSet* correspSet = project.getDataBlock<StereoVisionApp::CorrespondencesSet>(correspSetId);

    QVERIFY(img1 != nullptr);
    QVERIFY(img2 != nullptr);
    QVERIFY(cam != nullptr);
    QVERIFY(correspSet != nullptr);

    img1->assignCamera(cameraId);
    img2->assignCamera(cameraId);

    cam->setImHeight(480);
    cam->setImWidth(640);

    StereoVisionApp::floatParameter fLen(35);
    StereoVisionApp::floatParameter ppX(pFloatType(cam->imWidth())/2);
    StereoVisionApp::floatParameter ppY(pFloatType(cam->imHeight())/2);

    StereoVisionApp::floatParameter zero(0);

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

    StereoVision::Geometry::RigidBodyTransform<double> img12world(Eigen::Vector3d::Zero(), Eigen::Vector3d(10,0,0)); //cam2world
    StereoVision::Geometry::RigidBodyTransform<double> world2img1 = img12world.inverse();

    img1->setXCoord(img12world.t.x());
    img1->setYCoord(img12world.t.y());
    img1->setZCoord(img12world.t.z());

    img1->setXRot(img12world.r.x());
    img1->setYRot(img12world.r.y());
    img1->setZRot(img12world.r.z());

    StereoVision::Geometry::RigidBodyTransform<double> img22world(Eigen::Vector3d(0,-M_PI_2,0), Eigen::Vector3d(50,0,0)); //cam2world
    StereoVision::Geometry::RigidBodyTransform<double> world2img2 = img22world.inverse();

    img2->setXCoord(img22world.t.x());
    img2->setYCoord(img22world.t.y());
    img2->setZCoord(img22world.t.z());

    img2->setXRot(img22world.r.x());
    img2->setYRot(img22world.r.y());
    img2->setZRot(img22world.r.z());

    std::vector<Eigen::Vector3d> points =
        {
            Eigen::Vector3d{42,69,33},
            Eigen::Vector3d{33,27,50},
            Eigen::Vector3d{-6,-99,70},
            Eigen::Vector3d{-49,-28,12},
            Eigen::Vector3d{-69,-42,33}
        };

    for (Eigen::Vector3d const& point : points) {

        constexpr StereoVisionApp::Correspondences::Types UV = StereoVisionApp::Correspondences::UV;

        using UVCorresp = StereoVisionApp::Correspondences::Typed<UV>;

        Eigen::Vector3d ptImg1 = world2img1*point;

        Eigen::Vector2d uv1 = ptImg1.block<2,1>(0,0)/ptImg1.z() * fLen.value();
        uv1.x() += ppX.value();
        uv1.y() += ppY.value();

        Eigen::Vector3d ptImg2 = world2img2*point;

        Eigen::Vector2d uv2 = ptImg2.block<2,1>(0,0)/ptImg2.z() * fLen.value();
        uv2.x() += ppX.value();
        uv2.y() += ppY.value();

        UVCorresp uvCorresp1;
        uvCorresp1.blockId = image1Id;
        uvCorresp1.u = uv1.x();
        uvCorresp1.v = uv1.y();
        uvCorresp1.sigmaU = 1;
        uvCorresp1.sigmaV = 1;

        UVCorresp uvCorresp2;
        uvCorresp2.blockId = image2Id;
        uvCorresp2.u = uv2.x();
        uvCorresp2.v = uv2.y();
        uvCorresp2.sigmaU = 1;
        uvCorresp2.sigmaV = 1;

        correspSet->addCorrespondence({uvCorresp1, uvCorresp2});

    }


    StereoVisionApp::ModularSBASolver sbaSolver(&project);
    sbaSolver.setSilent(true);

    bool lmModuleAdded = sbaSolver.addModule(new StereoVisionApp::LandmarksSBAModule());
    bool imModuleAdded = sbaSolver.addModule(new StereoVisionApp::ImageAlignementSBAModule());
    bool correspModuleAdded = sbaSolver.addModule(new StereoVisionApp::CorrespondencesSetSBAModule());

    QVERIFY(lmModuleAdded);
    QVERIFY(imModuleAdded);
    QVERIFY(correspModuleAdded);

    bool initSuccess = sbaSolver.init();

    QVERIFY(initSuccess);

    QVERIFY(sbaSolver.itemIsObservable(image1Id));
    QVERIFY(sbaSolver.itemIsObservable(image2Id));

    ceres::Problem* problem = sbaSolver.ceresProblem();

    QVERIFY(problem != nullptr);

    //check something was added in the factor graph
    QVERIFY(problem->NumResidualBlocks() > 0);

    double cost;

    ceres::Problem::EvaluateOptions options;

    bool evaluateOk = problem->Evaluate(options, &cost, nullptr, nullptr, nullptr);

    QVERIFY(evaluateOk);

    QVERIFY(std::abs(cost) < 1e-6); //the problem should be initialized with a perfect solution

}


void TestSBASolver::image2RigidBody() {

    StereoVisionApp::ProjectFactory& pF = StereoVisionApp::ProjectFactory::defaultProjectFactory();

    std::unique_ptr<StereoVisionApp::Project> pPtr(pF.createProject(this)); //use unique ptr to ensure project is deleted at the end of test case

    QVERIFY(pPtr != nullptr);

    StereoVisionApp::Project& project = *pPtr;

    qint64 imageId = project.createDataBlock(StereoVisionApp::Image::staticMetaObject.className());
    qint64 cameraId = project.createDataBlock(StereoVisionApp::Camera::staticMetaObject.className());
    qint64 lcsId = project.createDataBlock(StereoVisionApp::LocalCoordinateSystem::staticMetaObject.className());
    qint64 correspSetId = project.createDataBlock(StereoVisionApp::CorrespondencesSet::staticMetaObject.className());

    StereoVisionApp::Image* img = project.getDataBlock<StereoVisionApp::Image>(imageId);
    StereoVisionApp::Camera* cam = project.getDataBlock<StereoVisionApp::Camera>(cameraId);
    StereoVisionApp::LocalCoordinateSystem* lcs = project.getDataBlock<StereoVisionApp::LocalCoordinateSystem>(lcsId);
    StereoVisionApp::CorrespondencesSet* correspSet = project.getDataBlock<StereoVisionApp::CorrespondencesSet>(correspSetId);

    QVERIFY(img != nullptr);
    QVERIFY(cam != nullptr);
    QVERIFY(lcs != nullptr);
    QVERIFY(correspSet != nullptr);

    img->assignCamera(cameraId);

    cam->setImHeight(480);
    cam->setImWidth(640);

    StereoVisionApp::floatParameter fLen(35);
    StereoVisionApp::floatParameter ppX(pFloatType(cam->imWidth())/2);
    StereoVisionApp::floatParameter ppY(pFloatType(cam->imHeight())/2);

    StereoVisionApp::floatParameter zero(0);

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

    StereoVision::Geometry::RigidBodyTransform<double> cam2world(Eigen::Vector3d::Random(), Eigen::Vector3d::Random()); //cam2world

    StereoVision::Geometry::RigidBodyTransform<double> world2cam = cam2world.inverse();

    img->setXCoord(cam2world.t.x());
    img->setYCoord(cam2world.t.y());
    img->setZCoord(cam2world.t.z());

    img->setXRot(cam2world.r.x());
    img->setYRot(cam2world.r.y());
    img->setZRot(cam2world.r.z());

    StereoVision::Geometry::RigidBodyTransform<double> lcs2world(Eigen::Vector3d::Random(), Eigen::Vector3d::Random()); //cam2world

    StereoVision::Geometry::RigidBodyTransform<double> world2lcs = lcs2world.inverse();

    lcs->setXCoord(lcs2world.t.x());
    lcs->setYCoord(lcs2world.t.y());
    lcs->setZCoord(lcs2world.t.z());

    lcs->setXRot(lcs2world.r.x());
    lcs->setYRot(lcs2world.r.y());
    lcs->setZRot(lcs2world.r.z());

    StereoVision::Geometry::RigidBodyTransform<double> cam2lcs = world2lcs*cam2world;

    std::vector<Eigen::Vector3d> points =
        {
            Eigen::Vector3d{42,69,33},
            Eigen::Vector3d{33,27,50},
            Eigen::Vector3d{-6,-99,70},
            Eigen::Vector3d{-49,-28,12},
            Eigen::Vector3d{-69,-42,33}
        };

    for (Eigen::Vector3d const& point : points) {

        constexpr StereoVisionApp::Correspondences::Types UV = StereoVisionApp::Correspondences::UV;
        constexpr StereoVisionApp::Correspondences::Types LcsPos = StereoVisionApp::Correspondences::XYZ;

        using UVCorresp = StereoVisionApp::Correspondences::Typed<UV>;
        using LcsCorresp = StereoVisionApp::Correspondences::Typed<LcsPos>;

        Eigen::Vector3d ptCam = point; //we start with the point in camera frame, to ensure z is positive.

        Eigen::Vector2d uv = ptCam.block<2,1>(0,0)/ptCam.z() * fLen.value();
        uv.x() += ppX.value();
        uv.y() += ppY.value();

        UVCorresp uvCorresp;
        uvCorresp.blockId = imageId;
        uvCorresp.u = uv.x();
        uvCorresp.v = uv.y();
        uvCorresp.sigmaU = 1;
        uvCorresp.sigmaV = 1;

        Eigen::Vector3d ptLcs = cam2lcs*point;

        LcsCorresp xyzCorresp;
        xyzCorresp.blockId = lcsId;
        xyzCorresp.x = ptLcs.x();
        xyzCorresp.y = ptLcs.y();
        xyzCorresp.z = ptLcs.z();

        correspSet->addCorrespondence({uvCorresp, xyzCorresp});
    }


    StereoVisionApp::ModularSBASolver sbaSolver(&project);
    sbaSolver.setSilent(true);

    bool imModuleAdded = sbaSolver.addModule(new StereoVisionApp::ImageAlignementSBAModule());
    bool lcsModuleAdded = sbaSolver.addModule(new StereoVisionApp::LocalCoordinateSystemSBAModule());
    bool correspModuleAdded = sbaSolver.addModule(new StereoVisionApp::CorrespondencesSetSBAModule());

    QVERIFY(imModuleAdded);
    QVERIFY(lcsModuleAdded);
    QVERIFY(correspModuleAdded);

    bool initSuccess = sbaSolver.init();

    QVERIFY(initSuccess);

    QVERIFY(sbaSolver.itemIsObservable(imageId));
    QVERIFY(sbaSolver.itemIsObservable(lcsId));

    ceres::Problem* problem = sbaSolver.ceresProblem();

    QVERIFY(problem != nullptr);

    //check something was added in the factor graph
    QVERIFY(problem->NumResidualBlocks() > 0);

    double cost;

    ceres::Problem::EvaluateOptions options;

    bool evaluateOk = problem->Evaluate(options, &cost, nullptr, nullptr, nullptr);

    QVERIFY(evaluateOk);

    QVERIFY(std::abs(cost) < 1e-6); //the problem should be initialized with a perfect solution

}
void TestSBASolver::rigidBody2RigidBody() {

    StereoVisionApp::ProjectFactory& pF = StereoVisionApp::ProjectFactory::defaultProjectFactory();

    std::unique_ptr<StereoVisionApp::Project> pPtr(pF.createProject(this)); //use unique ptr to ensure project is deleted at the end of test case

    QVERIFY(pPtr != nullptr);

    StereoVisionApp::Project& project = *pPtr;

    qint64 lcs1Id = project.createDataBlock(StereoVisionApp::LocalCoordinateSystem::staticMetaObject.className());
    qint64 lcs2Id = project.createDataBlock(StereoVisionApp::LocalCoordinateSystem::staticMetaObject.className());
    qint64 correspSetId = project.createDataBlock(StereoVisionApp::CorrespondencesSet::staticMetaObject.className());

    StereoVisionApp::LocalCoordinateSystem* lcs1 = project.getDataBlock<StereoVisionApp::LocalCoordinateSystem>(lcs1Id);
    StereoVisionApp::LocalCoordinateSystem* lcs2 = project.getDataBlock<StereoVisionApp::LocalCoordinateSystem>(lcs2Id);
    StereoVisionApp::CorrespondencesSet* correspSet = project.getDataBlock<StereoVisionApp::CorrespondencesSet>(correspSetId);

    QVERIFY(lcs1 != nullptr);
    QVERIFY(lcs2 != nullptr);
    QVERIFY(correspSet != nullptr);

    StereoVision::Geometry::RigidBodyTransform<double> lcs12world(Eigen::Vector3d::Random(), Eigen::Vector3d::Random()); //cam2world

    StereoVision::Geometry::RigidBodyTransform<double> world2lcs1 = lcs12world.inverse();

    lcs1->setXCoord(lcs12world.t.x());
    lcs1->setYCoord(lcs12world.t.y());
    lcs1->setZCoord(lcs12world.t.z());

    lcs1->setXRot(lcs12world.r.x());
    lcs1->setYRot(lcs12world.r.y());
    lcs1->setZRot(lcs12world.r.z());

    StereoVision::Geometry::RigidBodyTransform<double> lcs22world(Eigen::Vector3d::Random(), Eigen::Vector3d::Random()); //cam2world

    StereoVision::Geometry::RigidBodyTransform<double> world2lcs2 = lcs22world.inverse();

    lcs2->setXCoord(lcs22world.t.x());
    lcs2->setYCoord(lcs22world.t.y());
    lcs2->setZCoord(lcs22world.t.z());

    lcs2->setXRot(lcs22world.r.x());
    lcs2->setYRot(lcs22world.r.y());
    lcs2->setZRot(lcs22world.r.z());

    StereoVision::Geometry::RigidBodyTransform<double> lcs12lcs2 = world2lcs2*lcs12world;

    std::vector<Eigen::Vector3d> points =
        {
            Eigen::Vector3d{42,69,33},
            Eigen::Vector3d{33,27,50},
            Eigen::Vector3d{-6,-99,70},
            Eigen::Vector3d{-49,-28,12},
            Eigen::Vector3d{-69,-42,33}
        };

    for (Eigen::Vector3d const& point : points) {

        constexpr StereoVisionApp::Correspondences::Types LcsPos = StereoVisionApp::Correspondences::XYZ;

        using LcsCorresp = StereoVisionApp::Correspondences::Typed<LcsPos>;

        Eigen::Vector3d ptLcs1 = point;

        LcsCorresp xyzCorresp1;
        xyzCorresp1.blockId = lcs1Id;
        xyzCorresp1.x = ptLcs1.x();
        xyzCorresp1.y = ptLcs1.y();
        xyzCorresp1.z = ptLcs1.z();

        Eigen::Vector3d ptLcs2 = lcs12lcs2*point;

        LcsCorresp xyzCorresp2;
        xyzCorresp2.blockId = lcs2Id;
        xyzCorresp2.x = ptLcs2.x();
        xyzCorresp2.y = ptLcs2.y();
        xyzCorresp2.z = ptLcs2.z();

        correspSet->addCorrespondence({xyzCorresp1, xyzCorresp2});
    }


    StereoVisionApp::ModularSBASolver sbaSolver(&project);
    sbaSolver.setSilent(true);

    bool lcsModuleAdded = sbaSolver.addModule(new StereoVisionApp::LocalCoordinateSystemSBAModule());
    bool correspModuleAdded = sbaSolver.addModule(new StereoVisionApp::CorrespondencesSetSBAModule());

    QVERIFY(lcsModuleAdded);
    QVERIFY(correspModuleAdded);

    bool initSuccess = sbaSolver.init();

    QVERIFY(initSuccess);

    QVERIFY(sbaSolver.itemIsObservable(lcs1Id));
    QVERIFY(sbaSolver.itemIsObservable(lcs2Id));

    ceres::Problem* problem = sbaSolver.ceresProblem();

    QVERIFY(problem != nullptr);

    //check something was added in the factor graph
    QVERIFY(problem->NumResidualBlocks() > 0);

    double cost;

    ceres::Problem::EvaluateOptions options;

    bool evaluateOk = problem->Evaluate(options, &cost, nullptr, nullptr, nullptr);

    QVERIFY(evaluateOk);

    QVERIFY(std::abs(cost) < 1e-6); //the problem should be initialized with a perfect solution

}


void TestSBASolver::basicTrajectory_data() {

    QTest::addColumn<bool>("withStochasticProcesses");

    QTest::addRow("Without stochastic processes") << false;
    QTest::addRow("With stochastic processes") << true;

}

void TestSBASolver::basicTrajectory() {

    QFETCH(bool, withStochasticProcesses);

    StereoVisionApp::ProjectFactory& pF = StereoVisionApp::ProjectFactory::defaultProjectFactory();

    std::unique_ptr<StereoVisionApp::Project> pPtr(pF.createProject(this)); //use unique ptr to ensure project is deleted at the end of test case

    QVERIFY(pPtr != nullptr);

    StereoVisionApp::Project& project = *pPtr;

    qint64 trajectoryId = project.createDataBlock(StereoVisionApp::Trajectory::staticMetaObject.className());

    StereoVisionApp::Trajectory* traj = project.getDataBlock<StereoVisionApp::Trajectory>(trajectoryId);

    QVERIFY(traj != nullptr);

    configureStandardTrajectory(traj, ":/trajectories/zero_acc/sbet.csv", ":/trajectories/zero_acc/ins.csv");

    if (withStochasticProcesses) {

        constexpr double timeScale = 0.5;

        StereoVisionApp::Trajectory::InsStochasticProcessDef testProcess{.timeScale=timeScale,
                                                                         .betas=std::array<double,3>{0.5,0.5,0.5},
                                                                         .sigmas=std::array<double,3>{0.5,0.5,0.5}};

        traj->setGyroBiasStochasticProcesses({testProcess});
        traj->setGyroScaleStochasticProcesses({testProcess});
        traj->setAccBiasStochasticProcesses({testProcess});
        traj->setAccScaleStochasticProcesses({testProcess});

        traj->setEstAccelerometerScale(true);
        traj->setEstAccelerometerBias(true);
        traj->setEstGyroScale(true);
        traj->setEstGyroBias(true);
    }

    StereoVisionApp::StatusOptionalReturn<StereoVisionApp::Trajectory::TimeTrajectorySequence> optTrajData =
        traj->loadTrajectorySequence(); //time sequence of trajectory, corresponding to body to mapping

    QVERIFY(optTrajData.isValid());

    StereoVisionApp::ModularSBASolver sbaSolver(&project);
    sbaSolver.setSilent(true);

    StereoVisionApp::TrajectoryBaseSBAModule* trajSBAModule =
        new StereoVisionApp::TrajectoryBaseSBAModule(traj->getPreIntegrationTime());
    bool trajModuleAdded = sbaSolver.addModule(trajSBAModule);

    QVERIFY(trajModuleAdded);

    bool initSuccess = sbaSolver.init();

    QVERIFY(initSuccess);

    QVERIFY(sbaSolver.itemIsObservable(trajectoryId));

    ceres::Problem* problem = sbaSolver.ceresProblem();

    QVERIFY(problem != nullptr);

    //check something was added in the factor graph
    QVERIFY(problem->NumResidualBlocks() > 0);

    double cost;

    ceres::Problem::EvaluateOptions options;

    bool evaluateOk = problem->Evaluate(options, &cost, nullptr, nullptr, nullptr);

    QVERIFY(evaluateOk);

    qInfo() << "Cost = " << cost;
    QVERIFY(std::abs(cost) < 1e-6); //the problem should be initialized with a perfect solution

}

void TestSBASolver::movingBody2MovingBody() {

    StereoVisionApp::ProjectFactory& pF = StereoVisionApp::ProjectFactory::defaultProjectFactory();

    std::unique_ptr<StereoVisionApp::Project> pPtr(pF.createProject(this)); //use unique ptr to ensure project is deleted at the end of test case

    QVERIFY(pPtr != nullptr);

    StereoVisionApp::Project& project = *pPtr;

    qint64 lcs1Id = project.createDataBlock(StereoVisionApp::LocalCoordinateSystem::staticMetaObject.className());
    qint64 lcs2Id = project.createDataBlock(StereoVisionApp::LocalCoordinateSystem::staticMetaObject.className());
    qint64 trajectoryId = project.createDataBlock(StereoVisionApp::Trajectory::staticMetaObject.className());
    qint64 mountingId = project.createDataBlock(StereoVisionApp::Mounting::staticMetaObject.className());
    qint64 correspSetId = project.createDataBlock(StereoVisionApp::CorrespondencesSet::staticMetaObject.className());

    StereoVisionApp::LocalCoordinateSystem* lcs1 = project.getDataBlock<StereoVisionApp::LocalCoordinateSystem>(lcs1Id);
    StereoVisionApp::LocalCoordinateSystem* lcs2 = project.getDataBlock<StereoVisionApp::LocalCoordinateSystem>(lcs2Id);
    StereoVisionApp::Trajectory* traj = project.getDataBlock<StereoVisionApp::Trajectory>(trajectoryId);
    StereoVisionApp::Mounting* mounting = project.getDataBlock<StereoVisionApp::Mounting>(mountingId);
    StereoVisionApp::CorrespondencesSet* correspSet = project.getDataBlock<StereoVisionApp::CorrespondencesSet>(correspSetId);

    QVERIFY(lcs1 != nullptr);
    QVERIFY(lcs2 != nullptr);
    QVERIFY(traj != nullptr);
    QVERIFY(mounting != nullptr);
    QVERIFY(correspSet != nullptr);

    lcs1->assignTrajectory(trajectoryId);
    lcs2->assignTrajectory(trajectoryId);

    lcs1->assignMounting(mountingId);
    lcs2->assignMounting(mountingId);

    QCOMPARE(lcs1->getAssignedTrajectory(), traj);
    QCOMPARE(lcs2->getAssignedTrajectory(), traj);

    QCOMPARE(lcs1->getAssignedMounting(), mounting);
    QCOMPARE(lcs2->getAssignedMounting(), mounting);

    configureStandardTrajectory(traj, ":/trajectories/zero_acc/sbet.csv", ":/trajectories/zero_acc/ins.csv");

    mounting->setXCoord(0.42);
    mounting->setYCoord(0.33);
    mounting->setZCoord(0.69);

    mounting->setXRot(0.27);
    mounting->setYRot(0.13);
    mounting->setZRot(0.06);

    std::optional<StereoVision::Geometry::AffineTransform<float>> optMountingTransform = mounting->getTransform();

    QVERIFY(optMountingTransform.has_value());

    StereoVision::Geometry::RigidBodyTransform<double> sensor2body =
        StereoVision::Geometry::RigidBodyTransform<double>(optMountingTransform.value().cast<double>());

    //setup correspondences

    StereoVisionApp::StatusOptionalReturn<StereoVisionApp::Trajectory::TimeTrajectorySequence> optTrajData =
        traj->loadTrajectorySequence(); //time sequence of trajectory, corresponding to body to mapping

    QVERIFY(optTrajData.isValid());

    StereoVisionApp::Trajectory::TimeTrajectorySequence& trajData = optTrajData.value();

    struct PointsInfos {
        Eigen::Vector3d point;
        Eigen::Vector2d times;
    };

    std::vector<PointsInfos> points =
        {
            {Eigen::Vector3d{42,69,33}, Eigen::Vector2d{0.42,0.69}},
            {Eigen::Vector3d{33,27,50}, Eigen::Vector2d{0.33,0.27}},
            {Eigen::Vector3d{-6,-99,70}, Eigen::Vector2d{0.69,0.69}}, //same pose
            {Eigen::Vector3d{-49,-28,12}, Eigen::Vector2d{0.77,0.33}},
            {Eigen::Vector3d{-69,-42,33}, Eigen::Vector2d{0.42,0.42}} //same pose
        };

    for (PointsInfos const& infos : points) {

        Eigen::Vector3d const& point = infos.point;
        Eigen::Vector2d const& times = infos.times;

        constexpr StereoVisionApp::Correspondences::Types LidarVec = StereoVisionApp::Correspondences::XYZT;

        using LidarCorresp = StereoVisionApp::Correspondences::Typed<LidarVec>;

        auto interpolableBody1ToMapping = trajData.getValueAtTime(times[0]);
        StereoVision::Geometry::RigidBodyTransform<double> body1ToMapping =
            StereoVision::Geometry::interpolateRigidBodyTransformOnManifold(
            interpolableBody1ToMapping.weightLower,
            interpolableBody1ToMapping.valLower,
            interpolableBody1ToMapping.weightUpper,
            interpolableBody1ToMapping.valUpper);

        StereoVision::Geometry::RigidBodyTransform<double> sensor1ToMapping =
            body1ToMapping*sensor2body;

        auto interpolableBody2ToMapping = trajData.getValueAtTime(times[1]);
        StereoVision::Geometry::RigidBodyTransform<double> body2ToMapping =
            StereoVision::Geometry::interpolateRigidBodyTransformOnManifold(
                interpolableBody2ToMapping.weightLower,
                interpolableBody2ToMapping.valLower,
                interpolableBody2ToMapping.weightUpper,
                interpolableBody2ToMapping.valUpper);

        StereoVision::Geometry::RigidBodyTransform<double> sensor2ToMapping =
            body2ToMapping*sensor2body;

        StereoVision::Geometry::RigidBodyTransform<double> sensor1ToSensor2 =
            sensor2ToMapping.inverse()*sensor1ToMapping;

        Eigen::Vector3d ptSensor1 = point;

        LidarCorresp xyztCorresp1;
        xyztCorresp1.blockId = lcs1Id;
        xyztCorresp1.x = ptSensor1.x();
        xyztCorresp1.y = ptSensor1.y();
        xyztCorresp1.z = ptSensor1.z();
        xyztCorresp1.t = times[0];

        Eigen::Vector3d ptSensor2 = sensor1ToSensor2*point;

        LidarCorresp xyztCorresp2;
        xyztCorresp2.blockId = lcs2Id;
        xyztCorresp2.x = ptSensor2.x();
        xyztCorresp2.y = ptSensor2.y();
        xyztCorresp2.z = ptSensor2.z();
        xyztCorresp2.t = times[1];

        correspSet->addCorrespondence({xyztCorresp1, xyztCorresp2});
    }

    constexpr bool computeUncertainty = false;

    StereoVisionApp::ModularSBASolver sbaSolver(&project, computeUncertainty);
    sbaSolver.setSilent(true);

    StereoVisionApp::TrajectoryBaseSBAModule* trajSBAModule =
        new StereoVisionApp::TrajectoryBaseSBAModule(traj->getPreIntegrationTime());
    bool trajModuleAdded = sbaSolver.addModule(trajSBAModule);

    bool lcsModuleAdded = sbaSolver.addModule(new StereoVisionApp::LocalCoordinateSystemSBAModule());
    bool correspModuleAdded = sbaSolver.addModule(new StereoVisionApp::CorrespondencesSetSBAModule());
    bool mountingModuleAdded = sbaSolver.addModule(new StereoVisionApp::MountingsSBAModule());

    QVERIFY(trajModuleAdded);
    QVERIFY(lcsModuleAdded);
    QVERIFY(correspModuleAdded);
    QVERIFY(mountingModuleAdded);

    bool initSuccess = sbaSolver.init();

    QVERIFY(initSuccess);

    QVERIFY(sbaSolver.itemIsObservable(lcs1Id));
    QVERIFY(sbaSolver.itemIsObservable(lcs2Id));
    QVERIFY(sbaSolver.itemIsObservable(trajectoryId));

    ceres::Problem* problem = sbaSolver.ceresProblem();

    QVERIFY(problem != nullptr);

    //check something was added in the factor graph
    QVERIFY(problem->NumResidualBlocks() > 0);

    double cost;

    ceres::Problem::EvaluateOptions options;

    bool evaluateOk = problem->Evaluate(options, &cost, nullptr, nullptr, nullptr);

    QVERIFY(evaluateOk);

    QVERIFY(std::abs(cost) < 1e-6); //the problem should be initialized with a perfect solution
}

void TestSBASolver::testFixedWithEarthGravity() {    StereoVisionApp::ProjectFactory& pF = StereoVisionApp::ProjectFactory::defaultProjectFactory();

    StereoVisionApp::Project* pPtr = pF.createProject(this);

    QVERIFY(pPtr != nullptr);

    StereoVisionApp::Project& project = *pPtr;
    const char* ecefCRS = "EPSG:4978";

    project.setDefaultProjectCRS(ecefCRS);

    constexpr double EarthRadius = 6357000;

    StereoVision::Geometry::AffineTransform<double> ecef2Local(StereoVision::Geometry::rodriguezFormula(Eigen::Vector3d(0.27,-0.69,1.42)),
                                                               Eigen::Vector3d(0,0,-EarthRadius-3342));

    StereoVision::Geometry::AffineTransform<double> local2Ecef(ecef2Local.R.transpose(), -ecef2Local.R.transpose()*ecef2Local.t);

    project.setLocalCoordinateFrame(ecef2Local);

    qint64 trajectoryId = project.createDataBlock(StereoVisionApp::Trajectory::staticMetaObject.className());

    StereoVisionApp::Trajectory* traj = project.getDataBlock<StereoVisionApp::Trajectory>(trajectoryId);

    QVERIFY(traj != nullptr);

    StereoVisionApp::GeneratedTrajectory* genTraj = qobject_cast<StereoVisionApp::GeneratedTrajectory*>(traj);

    QVERIFY(genTraj != nullptr);

    genTraj->setPositionEpsg(ecefCRS);

    using TrajGeneratorInfos = StereoVisionApp::GeneratedTrajectory::TrajGeneratorInfos;

    constexpr double t0 = 0;
    constexpr double tf = 100;
    constexpr double dt = tf-t0;

    constexpr double rotationRate = 2*M_PI/(tf-t0);
    constexpr double circleRadius = 1000;

    double dtPos = 0.5;
    double dtAcc = 0.1;

    struct PositionCalculator {
        //position local
        static Eigen::Vector3d pos(double t) {
            return Eigen::Vector3d(0,0,0);
        }
        //speed local
        static Eigen::Vector3d speed(double t) {
            return Eigen::Vector3d(0,0,0);
        }
        //orientation local
        static Eigen::Vector3d orientation(double t) {
            return Eigen::Vector3d(0,0,0);
        }

        static Eigen::Vector3d coriolisEcefModel(Eigen::Vector3d const& ECEFPos, Eigen::Vector3d const& ECEFSpeed) {
            return Eigen::Vector3d(-2*ECEFSpeed[1]*StereoVisionApp::Geo::WGS84Ellipsoid::EarthRotationRate,
                                   2*ECEFSpeed[0]*StereoVisionApp::Geo::WGS84Ellipsoid::EarthRotationRate,
                                   0);
        }
    };

    Eigen::Vector3d earthRotation(0,0,StereoVisionApp::Geo::WGS84Ellipsoid::EarthRotationRate);
    Eigen::Vector3d earthRotationLocal = ecef2Local.R*earthRotation;

    genTraj->setInitialAndFinalTimes(t0,tf);

    genTraj->setAngularSpeedGenerator(TrajGeneratorInfos{[&earthRotationLocal] (double t) {
                                                             Eigen::Matrix3d Rlocal2Body = Eigen::Matrix3d::Identity();
                                                             Eigen::Vector3d ret = Rlocal2Body*earthRotationLocal;
                                                             return ret;
                                                         }, dtAcc}); //constant rotation rate
    genTraj->setAccelerationGenerator(TrajGeneratorInfos{[&local2Ecef] (double t) {
                                                             double acc = 0;
                                                             Eigen::Vector3d ecef = local2Ecef*PositionCalculator::pos(t);
                                                             Eigen::Vector3d speed = PositionCalculator::speed(t);
                                                             std::array<double,3> g = StereoVisionApp::Geo::WGS84Ellipsoid::gravityEcefModel(ecef); //g keep its amplitude and points toward center of the earth
                                                             Eigen::Vector3d coriolis = -PositionCalculator::coriolisEcefModel(ecef, speed);
                                                             Eigen::Matrix3d Rlocal2Body = Eigen::Matrix3d::Identity();
                                                             Eigen::Vector3d reorientedGravity = Rlocal2Body*local2Ecef.R.transpose()*Eigen::Vector3d(coriolis[0]+g[0],coriolis[1]+g[1],coriolis[2]+g[2]);
                                                             Eigen::Vector3d ret = Eigen::Vector3d(0,acc,0) +
                                                                                   reorientedGravity;
                                                             return ret;
                                                         }, dtAcc});

    genTraj->setPositionGenerator(TrajGeneratorInfos{[&local2Ecef] (double t) {
                                                         Eigen::Vector3d vec = local2Ecef*PositionCalculator::pos(t);
                                                         return vec;
                                                     }, dtPos});

    constexpr StereoVisionApp::Geo::TopocentricConvention topocentricConvention = StereoVisionApp::Geo::ENU;

    genTraj->setOrientationGenerator(TrajGeneratorInfos{[&local2Ecef] (double t) {
                                                            Eigen::Vector3d ecef = local2Ecef*PositionCalculator::pos(t);
                                                            Eigen::Matrix3d body2ecef = local2Ecef.R *
                                                                                        StereoVision::Geometry::rodriguezFormula<double>(PositionCalculator::orientation(t));
                                                            Eigen::Matrix3d topocentric2ecef = StereoVisionApp::Geo::localFrame2ECEFFromECEF(ecef, topocentricConvention);
                                                            return StereoVision::Geometry::inverseRodriguezFormula<double>(topocentric2ecef.transpose()*body2ecef); //body2topocentric
                                                        }, dtPos});
    traj->setOrientationAngleRepresentation(StereoVisionApp::Trajectory::AxisAngle);
    traj->setOrientationAngleUnits(StereoVisionApp::Trajectory::Radians);
    traj->setOrientationTopocentricConvention(topocentricConvention);



    traj->setPreIntegrationTime(0.5);
    traj->setGpsAccuracy(0.02);
    traj->setGyroAccuracy(0.1);
    traj->setAccAccuracy(0.5);


    constexpr bool computeUncertainty = false;

    StereoVisionApp::ModularSBASolver sbaSolver(&project, computeUncertainty);
    sbaSolver.setSilent(true);
    sbaSolver.setFuncTolerance(1e-8);
    sbaSolver.setParamsTolerance(1e-10);

    StereoVisionApp::TrajectoryBaseSBAModule* trajSBAModule =
        new StereoVisionApp::TrajectoryBaseSBAModule(traj->getPreIntegrationTime());
    bool trajModuleAdded = sbaSolver.addModule(trajSBAModule);

    QVERIFY(trajModuleAdded);

    bool initSuccess = sbaSolver.init();

    QVERIFY(initSuccess);

    QVERIFY(sbaSolver.itemIsObservable(trajectoryId));

    ceres::Problem* problem = sbaSolver.ceresProblem();

    QVERIFY(problem != nullptr);

    //check something was added in the factor graph
    QVERIFY(problem->NumResidualBlocks() > 0);



    bool optSuccess = sbaSolver.opt_step();

    QVERIFY(optSuccess);

    bool stdSuccess = sbaSolver.std_step();

    QVERIFY(stdSuccess);

    bool writeOk = sbaSolver.writeResults();

    QVERIFY(writeOk);

    bool writeStdOk = sbaSolver.writeUncertainty();

    QVERIFY(writeStdOk);

    sbaSolver.cleanup();

    StereoVisionApp::StatusOptionalReturn<StereoVisionApp::Trajectory::TimeTrajectorySequence> optoptTraj =
        traj->optimizedTrajectory();

    QVERIFY(optoptTraj.isValid());

    StereoVisionApp::Trajectory::TimeTrajectorySequence& optTraj = optoptTraj.value();

    double worseDelta = 0;
    double worseOrientationDelta = 0;
    Eigen::Vector3d meanPosDelta = Eigen::Vector3d::Zero();
    Eigen::Vector3d meanRotDelta = Eigen::Vector3d::Zero();

    double threshold = 1e-4;
    double thresholdOrientation = 1e-5;

    for (int i = 0; i < optTraj.nPoints(); i++) {
        auto& node = optTraj[i];
        Eigen::Vector3d expected = PositionCalculator::pos(optTraj[i].time);

        Eigen::Vector3d expectedOrientation = Eigen::Vector3d(0,0,0);

        Eigen::Vector3d dPos = node.val.t - expected;
        meanPosDelta += dPos;
        Eigen::Vector3d dRot = StereoVision::Geometry::inverseRodriguezFormula<double>(
            StereoVision::Geometry::rodriguezFormula<double>(node.val.r)*
            StereoVision::Geometry::rodriguezFormula<double>(-expectedOrientation));
        meanRotDelta += dRot;

        double delta = dPos.norm();
        double deltaOrientation = dRot.norm();

        if (delta >= threshold) {
            qWarning() << "About to fail with delta = " << delta << " (t = " << node.time << " position = "
                       << node.val.t.x() << " " << node.val.t.y() << " " << node.val.t.z()
                       << ", expected = " << expected.x() << " " << expected.y() << " " << expected.z() << ")";
        }

        if (deltaOrientation >= thresholdOrientation) {
            qWarning() << "About to fail with deltaOrientation = " << deltaOrientation << " (t = " << node.time << " orientation = "
                       << node.val.r.x() << " " << node.val.r.y() << " " << node.val.r.z()
                       << ", expected = " << expectedOrientation.x() << " " << expectedOrientation.y() << " " << expectedOrientation.z() << ")";
        }

        worseDelta = std::max(delta, worseDelta);
        worseOrientationDelta = std::max(deltaOrientation, worseOrientationDelta);
        QVERIFY(delta < threshold);
        QVERIFY(deltaOrientation < thresholdOrientation);
    }

    meanPosDelta /= optTraj.nPoints();
    meanRotDelta /= optTraj.nPoints();

    double biasThresholdOrientation = 1e-8;

    qInfo() << "Mean position alignement error: " << meanPosDelta.norm() << "m";
    qInfo() << "Mean orientation alignement error: " << meanRotDelta.norm() << "rad";

    qInfo() << "Worse position alignement error: " << worseDelta << "m";
    qInfo() << "Worse orientation alignement error: " << worseOrientationDelta << "rad";

    QVERIFY(meanPosDelta.norm() < biasThresholdOrientation);
    QVERIFY(meanRotDelta.norm() < biasThresholdOrientation);

}
void TestSBASolver::testWithEarthGravity() {

    StereoVisionApp::ProjectFactory& pF = StereoVisionApp::ProjectFactory::defaultProjectFactory();

    StereoVisionApp::Project* pPtr = pF.createProject(this);

    QVERIFY(pPtr != nullptr);

    StereoVisionApp::Project& project = *pPtr;
    const char* ecefCRS = "EPSG:4978";

    project.setDefaultProjectCRS(ecefCRS);

    constexpr double EarthRadius = 6357000;

    StereoVision::Geometry::AffineTransform<double> ecef2Local(StereoVision::Geometry::rodriguezFormula(Eigen::Vector3d(0.27,-0.69,1.42)),
                                                               Eigen::Vector3d(0,0,-EarthRadius-3342));

    StereoVision::Geometry::AffineTransform<double> local2Ecef(ecef2Local.R.transpose(), -ecef2Local.R.transpose()*ecef2Local.t);

    project.setLocalCoordinateFrame(ecef2Local);

    qint64 trajectoryId = project.createDataBlock(StereoVisionApp::Trajectory::staticMetaObject.className());

    StereoVisionApp::Trajectory* traj = project.getDataBlock<StereoVisionApp::Trajectory>(trajectoryId);

    QVERIFY(traj != nullptr);

    StereoVisionApp::GeneratedTrajectory* genTraj = qobject_cast<StereoVisionApp::GeneratedTrajectory*>(traj);

    QVERIFY(genTraj != nullptr);

    genTraj->setPositionEpsg(ecefCRS);

    using TrajGeneratorInfos = StereoVisionApp::GeneratedTrajectory::TrajGeneratorInfos;

    constexpr double t0 = 0;
    constexpr double tf = 100;
    constexpr double dt = tf-t0;

    constexpr double rotationRate = 2*M_PI/(tf-t0);
    constexpr double circleRadius = 1000;

    double dtPos = 0.5;
    double dtAcc = 0.1;

    struct PositionCalculator {
        //position local
        static Eigen::Vector3d pos(double t) {
            double x = circleRadius*std::cos(t/dt * 2*M_PI);
            double y = circleRadius*std::sin(t/dt * 2*M_PI);
            double z = 0;
            return Eigen::Vector3d(x,y,z);
        }
        //speed local
        static Eigen::Vector3d speed(double t) {
            double x = -circleRadius*std::sin(t/dt * 2*M_PI)/dt * 2*M_PI;
            double y = circleRadius*std::cos(t/dt * 2*M_PI)/dt * 2*M_PI;
            double z = 0;
            return Eigen::Vector3d(x,y,z);
        }
        //orientation local
        static Eigen::Vector3d orientation(double t) {
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

    genTraj->setAngularSpeedGenerator(TrajGeneratorInfos{[&earthRotationLocal] (double t) {
                                                             double omega = rotationRate;
                                                             Eigen::Matrix3d Rlocal2Body = StereoVision::Geometry::rodriguezFormula<double>(-PositionCalculator::orientation(t));
                                                             Eigen::Vector3d ret = (Rlocal2Body*earthRotationLocal)+Eigen::Vector3d(0,0,omega);
                                                             return ret;
                                                         }, dtAcc}); //constant rotation rate
    genTraj->setAccelerationGenerator(TrajGeneratorInfos{[&local2Ecef] (double t) {
                                                             double acc = -circleRadius*rotationRate*rotationRate;
                                                             Eigen::Vector3d ecef = local2Ecef*PositionCalculator::pos(t);
                                                             Eigen::Vector3d speed = local2Ecef.R*PositionCalculator::speed(t);
                                                             std::array<double,3> gArray = StereoVisionApp::Geo::WGS84Ellipsoid::gravityEcefModel(ecef); //g keep its amplitude and points toward center of the earth
                                                             Eigen::Vector3d g(gArray[0],gArray[1],gArray[2]);
                                                             Eigen::Vector3d coriolis = PositionCalculator::coriolisEcefModel(ecef, speed);
                                                             Eigen::Matrix3d Rlocal2Body = StereoVision::Geometry::rodriguezFormula<double>(-PositionCalculator::orientation(t));

                                                             Eigen::Vector3d gLocal = local2Ecef.R.transpose()*g;
                                                             Eigen::Vector3d gBody = Rlocal2Body*gLocal;

                                                             Eigen::Vector3d coriolisLocal = local2Ecef.R.transpose()*coriolis;
                                                             Eigen::Vector3d coriolisBody = Rlocal2Body*coriolisLocal;

                                                             Eigen::Vector3d reorientedGravity = coriolisBody+gBody;

                                                             Eigen::Vector3d ret = Eigen::Vector3d(0,acc,0) +
                                                                                    reorientedGravity;
                                                             return ret;
                                                         }, dtAcc});

    genTraj->setPositionGenerator(TrajGeneratorInfos{[&local2Ecef] (double t) {
                                                         Eigen::Vector3d vec = local2Ecef*PositionCalculator::pos(t);
                                                         return vec;
                                                     }, dtPos});

    constexpr StereoVisionApp::Geo::TopocentricConvention topocentricConvention = StereoVisionApp::Geo::ENU;

    genTraj->setOrientationGenerator(TrajGeneratorInfos{[&local2Ecef] (double t) {
                                                            Eigen::Vector3d ecef = local2Ecef*PositionCalculator::pos(t);
                                                            Eigen::Matrix3d body2ecef = local2Ecef.R *
                                                                StereoVision::Geometry::rodriguezFormula<double>(PositionCalculator::orientation(t));
                                                            Eigen::Matrix3d topocentric2ecef = StereoVisionApp::Geo::localFrame2ECEFFromECEF(ecef, topocentricConvention);
                                                            return StereoVision::Geometry::inverseRodriguezFormula<double>(topocentric2ecef.transpose()*body2ecef); //body2topocentric
                                                        }, dtPos});
    traj->setOrientationAngleRepresentation(StereoVisionApp::Trajectory::AxisAngle);
    traj->setOrientationAngleUnits(StereoVisionApp::Trajectory::Radians);
    traj->setOrientationTopocentricConvention(topocentricConvention);

    auto trajectoryDataOpt = traj->loadTrajectoryProjectLocalFrameSequence();

    QVERIFY2(trajectoryDataOpt.isValid(), trajectoryDataOpt.errorMessage());

    auto& trajectoryData = trajectoryDataOpt.value();

    QVERIFY(trajectoryData.nPoints() > 0);

    for (int i = 0; i < trajectoryData.nPoints(); i++) {
        auto& body2local = trajectoryData[i];
        Eigen::Vector3d posExpected = PositionCalculator::pos(body2local.time);
        Eigen::Vector3d orientationExpected = PositionCalculator::orientation(body2local.time);

        Eigen::Vector3d orientationDelta = StereoVision::Geometry::inverseRodriguezFormula<double>(StereoVision::Geometry::rodriguezFormula<double>(body2local.val.r)*
                                                                                           StereoVision::Geometry::rodriguezFormula<double>(-orientationExpected));

        for (int i = 0; i < 3; i++) {
            constexpr double posThresh = 1e-8;
            constexpr double rThreshold = 1e-6;
            double dPos = body2local.val.t[i] - posExpected[i];
            double dRot = orientationDelta[i];
            if (std::abs(dPos) >= posThresh) {
                qWarning() << "Pos misalignement at time: " << body2local.time << " axis: " << i << " original = " << body2local.val.t[i] << " expected = " << posExpected[i];
            }
            if (std::abs(dRot) >= rThreshold) {
                qWarning() << "Rot misalignement at time: " << body2local.time << " axis: " << i << " delta = " << orientationDelta[i];
            }
            QVERIFY(std::abs(dPos) < posThresh);
            QVERIFY(std::abs(dRot) < rThreshold);
        }
    }

    traj->setPreIntegrationTime(0.5);
    traj->setGpsAccuracy(0.02);
    traj->setGyroAccuracy(0.1);
    traj->setAccAccuracy(0.5);


    constexpr bool computeUncertainty = false;

    StereoVisionApp::ModularSBASolver sbaSolver(&project, computeUncertainty);
    sbaSolver.setSilent(true);
    sbaSolver.setFuncTolerance(1e-8);
    sbaSolver.setParamsTolerance(1e-10);

    StereoVisionApp::TrajectoryBaseSBAModule* trajSBAModule =
        new StereoVisionApp::TrajectoryBaseSBAModule(traj->getPreIntegrationTime());
    bool trajModuleAdded = sbaSolver.addModule(trajSBAModule);

    QVERIFY(trajModuleAdded);

    bool initSuccess = sbaSolver.init();

    QVERIFY(initSuccess);

    QVERIFY(sbaSolver.itemIsObservable(trajectoryId));

    ceres::Problem* problem = sbaSolver.ceresProblem();

    QVERIFY(problem != nullptr);

    //check something was added in the factor graph
    QVERIFY(problem->NumResidualBlocks() > 0);



    bool optSuccess = sbaSolver.opt_step();

    QVERIFY(optSuccess);

    bool stdSuccess = sbaSolver.std_step();

    QVERIFY(stdSuccess);

    bool writeOk = sbaSolver.writeResults();

    QVERIFY(writeOk);

    bool writeStdOk = sbaSolver.writeUncertainty();

    QVERIFY(writeStdOk);

    sbaSolver.cleanup();

    StereoVisionApp::StatusOptionalReturn<StereoVisionApp::Trajectory::TimeTrajectorySequence> optoptTraj =
        traj->optimizedTrajectory();

    QVERIFY(optoptTraj.isValid());

    StereoVisionApp::Trajectory::TimeTrajectorySequence& optTraj = optoptTraj.value();

    double worseDelta = 0;
    double worseOrientationDelta = 0;
    Eigen::Vector3d meanPosDelta = Eigen::Vector3d::Zero();
    Eigen::Vector3d meanRotDelta = Eigen::Vector3d::Zero();

    double threshold = 1e-6;
    double thresholdOrientation = 1e-5;

    for (int i = 0; i < optTraj.nPoints(); i++) {
        auto& node = optTraj[i];
        Eigen::Vector3d expected = PositionCalculator::pos(optTraj[i].time);

        Eigen::Vector3d expectedOrientation = PositionCalculator::orientation(optTraj[i].time);

        Eigen::Vector3d dPos = node.val.t - expected;
        meanPosDelta += dPos;
        Eigen::Vector3d dRot = StereoVision::Geometry::inverseRodriguezFormula<double>(
            StereoVision::Geometry::rodriguezFormula<double>(node.val.r)*
            StereoVision::Geometry::rodriguezFormula<double>(-expectedOrientation));
        meanRotDelta += dRot;

        double delta = dPos.norm();
        double deltaOrientation = dRot.norm();

        if (delta >= threshold) {
            qWarning() << "About to fail with delta = " << delta << " (t = " << node.time << " position = "
                    << node.val.t.x() << " " << node.val.t.y() << " " << node.val.t.z()
                    << ", expected = " << expected.x() << " " << expected.y() << " " << expected.z() << ")";
        }

        if (deltaOrientation >= thresholdOrientation) {
            qWarning() << "About to fail with deltaOrientation = " << deltaOrientation << " (t = " << node.time << " orientation = "
                       << node.val.r.x() << " " << node.val.r.y() << " " << node.val.r.z()
                       << ", expected = " << expectedOrientation.x() << " " << expectedOrientation.y() << " " << expectedOrientation.z() << ")";
        }

        worseDelta = std::max(delta, worseDelta);
        worseOrientationDelta = std::max(deltaOrientation, worseOrientationDelta);
        QVERIFY(delta < threshold);
        QVERIFY(deltaOrientation < thresholdOrientation);
    }

    meanPosDelta /= optTraj.nPoints();
    meanRotDelta /= optTraj.nPoints();

    double biasThresholdOrientation = 1e-6;

    qInfo() << "Mean position alignement error: " << meanPosDelta.norm() << "m";
    qInfo() << "Mean orientation alignement error: " << meanRotDelta.norm() << "rad";

    qInfo() << "Worse position alignement error: " << worseDelta << "m";
    qInfo() << "Worse orientation alignement error: " << worseOrientationDelta << "rad";

    QVERIFY(meanPosDelta.norm() < biasThresholdOrientation);
    QVERIFY(meanRotDelta.norm() < biasThresholdOrientation);

}
void TestSBASolver::testWaypointTrajectoryArpette() {

    StereoVisionApp::ProjectFactory& pF = StereoVisionApp::ProjectFactory::defaultProjectFactory();

    StereoVisionApp::Project* pPtr = pF.createProject(this);

    QVERIFY(pPtr != nullptr);

    StereoVisionApp::Project& project = *pPtr;
    const char* ecefCRS = "EPSG:4978";

    project.setDefaultProjectCRS(ecefCRS);

    std::vector<std::array<double,3>> waypointsLatLonHeight = {
        {46.03492177897298, 7.087220353061962, 1878.258324937554},
        {46.02146192956894, 7.098090353995183, 2151.688920219349},
        {46.02236597934959, 7.100801500967546, 2079.217496895602},
        {46.03565599595825, 7.089615022981861, 1799.531696490634}
    };

    std::vector<std::array<double,3>> waypointsECEF(waypointsLatLonHeight.size());

    Eigen::Vector3d meanPos = Eigen::Vector3d::Zero();

    for (size_t i = 0; i < waypointsLatLonHeight.size(); i++) {
        waypointsECEF[i] = StereoVisionApp::Geo::WGS84Ellipsoid::LatLonHeight2ECEF(waypointsLatLonHeight[i]);
        for (int j = 0; j < 3; j++) {
            meanPos[j] += waypointsECEF[i][j];
        }
    }

    meanPos /= waypointsECEF.size();

    qInfo() << "Mean pos: " << meanPos.x() << " " << meanPos.y() << " " << meanPos.z();

    StereoVision::Geometry::AffineTransform<double> ecef2Local(Eigen::Matrix3d::Identity(),
                                                               -meanPos);

    StereoVision::Geometry::AffineTransform<double> local2Ecef(ecef2Local.R.transpose(), -ecef2Local.R.transpose()*ecef2Local.t);

    project.setLocalCoordinateFrame(ecef2Local);

    qint64 trajectoryId = project.createDataBlock(StereoVisionApp::Trajectory::staticMetaObject.className());

    StereoVisionApp::Trajectory* traj = project.getDataBlock<StereoVisionApp::Trajectory>(trajectoryId);

    QVERIFY(traj != nullptr);

    StereoVisionApp::GeneratedTrajectory* genTraj = qobject_cast<StereoVisionApp::GeneratedTrajectory*>(traj);

    QVERIFY(genTraj != nullptr);

    genTraj->setPositionEpsg(ecefCRS);

    int freqGps = 2; // gps sampling freq Hz
    int freqIns = 10; // ins sampling freq Hz

    int tickGPS = freqIns/freqGps;

    double dtPos = 1./freqGps;
    double dtAcc = 1./freqIns;
    double dt = std::min(dtAcc, dtPos);

    QVERIFY(dt > 0);
    QVERIFY(dtPos*freqGps > 0.99);
    QVERIFY(dtAcc*freqIns > 0.99);

    double speed = 5; // 50 m/s
    double smoothTime = 4; // 0.5 s

    StereoVisionApp::Simulation::WayPointsEcefTrajectoryFunctor* trajFunctor =
        new StereoVisionApp::Simulation::WayPointsEcefTrajectoryFunctor(waypointsECEF,speed,smoothTime);

    StereoVisionApp::Simulation::EcefTrajectoryInertialInstrumentsSimulator staticTrajSimulator(trajFunctor);

    auto wayPointsInternal = trajFunctor->orientationWaypoints();
    double t0 = wayPointsInternal.times.front();
    double tf = wayPointsInternal.times.back();

    int nSamples = std::ceil((tf-t0)/dt);
    int nSamplesGps = std::ceil((tf-t0)/dtPos);

    double currentT = t0;
    int tickCount = 0;

    std::vector<StereoVisionApp::Trajectory::TimeCartesianBlock> posCache;
    std::vector<StereoVisionApp::Trajectory::TimeCartesianBlock> orientCache;
    #ifndef NDEBUG
        std::vector<StereoVisionApp::Trajectory::TimeCartesianBlock> ecefOrientCache;
    #endif
    StereoVisionApp::Trajectory::RawGpsData gpsCache;
    std::vector<StereoVisionApp::Trajectory::TimeCartesianBlock> gyroCache;
    std::vector<StereoVisionApp::Trajectory::TimeCartesianBlock> accCache;

    posCache.reserve(nSamplesGps);
    orientCache.reserve(nSamplesGps);
    #ifndef NDEBUG
        ecefOrientCache.reserve(nSamplesGps);
    #endif
    gpsCache.position = std::vector<StereoVisionApp::Trajectory::TimeCartesianBlock>();
    gpsCache.position->reserve(nSamplesGps);
    /*gpsCache.velocities = std::vector<StereoVisionApp::Trajectory::TimeCartesianBlock>();
    gpsCache.velocities->reserve(nSamplesGps);*/
    gpsCache.velocities = std::nullopt;
    gpsCache.posSigma = std::nullopt;
    gpsCache.speedSigma = std::nullopt;

    gyroCache.reserve(nSamples);
    accCache.reserve(nSamples);

    constexpr StereoVisionApp::Geo::TopocentricConvention topocentricConvention = StereoVisionApp::Geo::ENU;

    while (currentT < tf) {

        if (tickCount % tickGPS == 0) {
            StereoVision::Geometry::RigidBodyTransform<double> body2ecef = staticTrajSimulator.body2ecef(currentT);

            posCache.push_back(StereoVisionApp::Trajectory::TimeCartesianBlock{.time=currentT,.val=body2ecef.t});

            Eigen::Matrix3d topocentric2ecef = StereoVisionApp::Geo::localFrame2ECEFFromECEF(body2ecef.t, topocentricConvention);
            Eigen::Vector3d orient = StereoVision::Geometry::inverseRodriguezFormula<double>(topocentric2ecef.transpose()*
                                                                    StereoVision::Geometry::rodriguezFormula<double>(body2ecef.r)); //body2topocentric
            #ifndef NDEBUG
                ecefOrientCache.push_back(StereoVisionApp::Trajectory::TimeCartesianBlock{.time=currentT,.val=body2ecef.r});
            #endif
            orientCache.push_back(StereoVisionApp::Trajectory::TimeCartesianBlock{.time=currentT,.val=orient});

            Eigen::Vector3d gps = staticTrajSimulator.gps(currentT);
            //Eigen::Vector3d gpsVelocity = staticTrajSimulator.gpsVelocity(currentT);

            gpsCache.position->push_back(StereoVisionApp::Trajectory::TimeCartesianBlock{.time=currentT,.val=gps});
            //gpsCache.velocities->push_back(StereoVisionApp::Trajectory::TimeCartesianBlock{.time=currentT,.val=gpsVelocity});
        }
        Eigen::Vector3d gyro = staticTrajSimulator.gyro(currentT);
        Eigen::Vector3d acc = staticTrajSimulator.acc(currentT);

        gyroCache.push_back(StereoVisionApp::Trajectory::TimeCartesianBlock{.time=currentT,.val=gyro});
        accCache.push_back(StereoVisionApp::Trajectory::TimeCartesianBlock{.time=currentT,.val=acc});

        currentT += dt;
        tickCount++;
    }

    genTraj->setCachedRawPos(posCache);
    genTraj->setCachedRawOrient(orientCache);
    genTraj->setCachedRawGps(gpsCache);
    genTraj->setCachedRawGyro(gyroCache);
    genTraj->setCachedRawAcc(accCache);

    traj->setOrientationAngleRepresentation(StereoVisionApp::Trajectory::AxisAngle);
    traj->setOrientationAngleUnits(StereoVisionApp::Trajectory::Radians);
    traj->setOrientationTopocentricConvention(topocentricConvention);

    traj->setPreIntegrationTime(0.5);
    traj->setGpsAccuracy(0.02);
    traj->setGyroAccuracy(0.1);
    traj->setAccAccuracy(0.5);

    constexpr bool computeUncertainty = false;

    StereoVisionApp::ModularSBASolver sbaSolver(&project, computeUncertainty);
    sbaSolver.setSilent(true);
    sbaSolver.setFuncTolerance(1e-8);
    sbaSolver.setParamsTolerance(1e-10);
    sbaSolver.setOptimizationSteps(1);
    sbaSolver.enableLogging("/home/laurent/Pictures/steviapp_test_projects/waypoints_test_project/logging/", "unit_test");

    StereoVisionApp::TrajectoryBaseSBAModule* trajSBAModule =
        new StereoVisionApp::TrajectoryBaseSBAModule(traj->getPreIntegrationTime());
    bool trajModuleAdded = sbaSolver.addModule(trajSBAModule);
    QVERIFY(trajModuleAdded);

    bool initSuccess = sbaSolver.init();

    QVERIFY(initSuccess);

    QVERIFY(sbaSolver.itemIsObservable(trajectoryId));

    ceres::Problem* problem = sbaSolver.ceresProblem();

    QVERIFY(problem != nullptr);

    //check something was added in the factor graph
    QVERIFY(problem->NumResidualBlocks() > 0);



    bool optSuccess = sbaSolver.opt_step();

    QVERIFY(optSuccess);

    bool stdSuccess = sbaSolver.std_step();

    QVERIFY(stdSuccess);

    bool writeOk = sbaSolver.writeResults();

    QVERIFY(writeOk);

    bool writeStdOk = sbaSolver.writeUncertainty();

    QVERIFY(writeStdOk);

    sbaSolver.cleanup();

    StereoVisionApp::StatusOptionalReturn<StereoVisionApp::Trajectory::TimeTrajectorySequence> optoptTraj =
        traj->optimizedTrajectory();

    QVERIFY(optoptTraj.isValid());

    StereoVisionApp::Trajectory::TimeTrajectorySequence& optTraj = optoptTraj.value();

    double worseDelta = 0;
    double worseOrientationDelta = 0;
    Eigen::Vector3d meanPosDelta = Eigen::Vector3d::Zero();
    Eigen::Vector3d meanRotDelta = Eigen::Vector3d::Zero();

    double threshold = 1e-1;
    double thresholdOrientation = 1e-1;

    StereoVisionApp::Simulation::WayPointsEcefTrajectoryFunctor gtTraj(waypointsECEF,speed,smoothTime);

    for (int i = 0; i < optTraj.nPoints(); i++) {
        auto& node = optTraj[i];

        using D2Jet = ceres::Jet<ceres::Jet<double,1>,1>;

        D2Jet jet_t;
        jet_t.a.a = optTraj[i].time;
        jet_t.a.v[0] = 1;
        jet_t.v[0].a = 1;
        StereoVision::Geometry::RigidBodyTransform<D2Jet> trajPosEcef = gtTraj.trajectory(jet_t);
        StereoVision::Geometry::RigidBodyTransform<double> trajPos;

        for (int i = 0; i < 3; i++) {
            trajPos.r[i] = trajPosEcef.r[i].a.a;
            trajPos.t[i] = trajPosEcef.t[i].a.a;
        }

        Eigen::Vector3d expected(trajPos.t);

        expected = ecef2Local*expected;

        Eigen::Matrix3d topocentric2ecef = StereoVisionApp::Geo::localFrame2ECEFFromECEF(trajPos.t, topocentricConvention);
        Eigen::Vector3d orientLocal = StereoVision::Geometry::inverseRodriguezFormula<double>(topocentric2ecef.transpose()*
                                                                                              StereoVision::Geometry::rodriguezFormula<double>(trajPos.r));

        Eigen::Vector3d expectedOrientation = StereoVision::Geometry::inverseRodriguezFormula<double>(topocentric2ecef*
                                                                                                      StereoVision::Geometry::rodriguezFormula<double>(orientLocal));



        Eigen::Vector3d dPos = node.val.t - expected;
        meanPosDelta += dPos;
        Eigen::Vector3d dRot = StereoVision::Geometry::inverseRodriguezFormula<double>(
            StereoVision::Geometry::rodriguezFormula<double>(node.val.r)*
            StereoVision::Geometry::rodriguezFormula<double>(-expectedOrientation));
        meanRotDelta += dRot;

        double delta = dPos.norm();
        double deltaOrientation = dRot.norm();

        if (delta >= threshold) {
            qWarning() << "About to fail with delta = " << delta << " (t = " << node.time << " position = "
                       << node.val.t.x() << " " << node.val.t.y() << " " << node.val.t.z()
                       << ", expected = " << expected.x() << " " << expected.y() << " " << expected.z() << ")";
        }

        if (deltaOrientation >= thresholdOrientation) {
            qWarning() << "About to fail with deltaOrientation = " << deltaOrientation << " (t = " << node.time << " orientation = "
                       << node.val.r.x() << " " << node.val.r.y() << " " << node.val.r.z()
                       << ", expected = " << expectedOrientation.x() << " " << expectedOrientation.y() << " " << expectedOrientation.z() << ")";
        }

        worseDelta = std::max(delta, worseDelta);
        worseOrientationDelta = std::max(deltaOrientation, worseOrientationDelta);
        QVERIFY(delta < threshold);
        QVERIFY(deltaOrientation < thresholdOrientation);
    }

    meanPosDelta /= optTraj.nPoints();
    meanRotDelta /= optTraj.nPoints();

    double biasThresholdPosition = 1e-3;
    double biasThresholdOrientation = 1e-4;

    qInfo() << "Mean position alignement error: " << meanPosDelta.norm() << "m";
    qInfo() << "Mean orientation alignement error: " << meanRotDelta.norm() << "rad";

    qInfo() << "Worse position alignement error: " << worseDelta << "m";
    qInfo() << "Worse orientation alignement error: " << worseOrientationDelta << "rad";

    QVERIFY(meanPosDelta.norm() < biasThresholdPosition);
    QVERIFY(meanRotDelta.norm() < biasThresholdOrientation);
}


void TestSBASolver::testCameras2Trajectory() {

    StereoVisionApp::ProjectFactory& pF = StereoVisionApp::ProjectFactory::defaultProjectFactory();

    StereoVisionApp::Project* pPtr = pF.createProject(this);

    QVERIFY(pPtr != nullptr);

    StereoVisionApp::Project& project = *pPtr;
    const char* ecefCRS = "EPSG:4978";

    project.setDefaultProjectCRS(ecefCRS);

    std::vector<std::array<double,3>> waypointsLatLonHeight = {
        {46.03492177897298, 7.087220353061962, 1878.258324937554},
        {46.02146192956894, 7.098090353995183, 2151.688920219349}
    };

    std::vector<std::array<double,3>> waypointsECEF(waypointsLatLonHeight.size());

    Eigen::Vector3d meanPos = Eigen::Vector3d::Zero();

    for (size_t i = 0; i < waypointsLatLonHeight.size(); i++) {
        waypointsECEF[i] = StereoVisionApp::Geo::WGS84Ellipsoid::LatLonHeight2ECEF(waypointsLatLonHeight[i]);
        for (int j = 0; j < 3; j++) {
            meanPos[j] += waypointsECEF[i][j];
        }
    }

    meanPos /= waypointsECEF.size();

    qInfo() << "Mean pos: " << meanPos.x() << " " << meanPos.y() << " " << meanPos.z();

    StereoVision::Geometry::AffineTransform<double> ecef2Local(Eigen::Matrix3d::Identity(),
                                                               -meanPos);

    StereoVision::Geometry::RigidBodyTransform<double> ecef2LocalRigid(Eigen::Vector3d::Zero(), ecef2Local.t);

    StereoVision::Geometry::AffineTransform<double> local2Ecef(ecef2Local.R.transpose(), -ecef2Local.R.transpose()*ecef2Local.t);

    project.setLocalCoordinateFrame(ecef2Local);

    qint64 trajectoryId = project.createDataBlock(StereoVisionApp::Trajectory::staticMetaObject.className());

    StereoVisionApp::Trajectory* traj = project.getDataBlock<StereoVisionApp::Trajectory>(trajectoryId);

    QVERIFY(traj != nullptr);

    StereoVisionApp::GeneratedTrajectory* genTraj = qobject_cast<StereoVisionApp::GeneratedTrajectory*>(traj);

    QVERIFY(genTraj != nullptr);

    genTraj->setPositionEpsg(ecefCRS);

    int freqGps = 2; // gps sampling freq Hz
    int freqIns = 10; // ins sampling freq Hz

    int tickGPS = freqIns/freqGps;

    double dtPos = 1./freqGps;
    double dtAcc = 1./freqIns;
    double dt = std::min(dtAcc, dtPos);

    QVERIFY(dt > 0);
    QVERIFY(dtPos*freqGps > 0.99);
    QVERIFY(dtAcc*freqIns > 0.99);

    double speed = 24;
    double smoothTime = 4; // 0.5 s

    StereoVisionApp::Simulation::WayPointsEcefTrajectoryFunctor* trajFunctor =
        new StereoVisionApp::Simulation::WayPointsEcefTrajectoryFunctor(waypointsECEF,speed,smoothTime);

    StereoVisionApp::Simulation::EcefTrajectoryInertialInstrumentsSimulator staticTrajSimulator(trajFunctor);

    auto wayPointsInternal = trajFunctor->positionWaypoints();
    double t0 = wayPointsInternal.times.front();
    double tf = wayPointsInternal.times.back();


    QVERIFY(std::abs(t0) < 1e-5); //T0 = 0
    if (!(tf < 75 and tf > 70)) {
        qWarning() << "Unexpected final time of: " << tf;
    }
    QVERIFY(tf < 75 and tf > 70); //Tf is around 74

    int nSamples = std::ceil((tf-t0)/dt);
    int nSamplesGps = std::ceil((tf-t0)/dtPos);

    double currentT = t0;
    int tickCount = 0;

    std::vector<StereoVisionApp::Trajectory::TimeCartesianBlock> posCache;
    std::vector<StereoVisionApp::Trajectory::TimeCartesianBlock> orientCache;
#ifndef NDEBUG
    std::vector<StereoVisionApp::Trajectory::TimeCartesianBlock> ecefOrientCache;
#endif
    StereoVisionApp::Trajectory::RawGpsData gpsCache;
    std::vector<StereoVisionApp::Trajectory::TimeCartesianBlock> gyroCache;
    std::vector<StereoVisionApp::Trajectory::TimeCartesianBlock> accCache;

    posCache.reserve(nSamplesGps);
    orientCache.reserve(nSamplesGps);
#ifndef NDEBUG
    ecefOrientCache.reserve(nSamplesGps);
#endif
    gpsCache.position = std::vector<StereoVisionApp::Trajectory::TimeCartesianBlock>();
    gpsCache.position->reserve(nSamplesGps);
    /*gpsCache.velocities = std::vector<StereoVisionApp::Trajectory::TimeCartesianBlock>();
    gpsCache.velocities->reserve(nSamplesGps);*/
    gpsCache.velocities = std::nullopt;
    gpsCache.posSigma = std::nullopt;
    gpsCache.speedSigma = std::nullopt;

    gyroCache.reserve(nSamples);
    accCache.reserve(nSamples);

    constexpr StereoVisionApp::Geo::TopocentricConvention topocentricConvention = StereoVisionApp::Geo::ENU;

    while (currentT < tf) {

        if (tickCount % tickGPS == 0) {
            StereoVision::Geometry::RigidBodyTransform<double> body2ecef = staticTrajSimulator.body2ecef(currentT);

            posCache.push_back(StereoVisionApp::Trajectory::TimeCartesianBlock{.time=currentT,.val=body2ecef.t});

            Eigen::Matrix3d topocentric2ecef = StereoVisionApp::Geo::localFrame2ECEFFromECEF(body2ecef.t, topocentricConvention);
            Eigen::Vector3d orient = StereoVision::Geometry::inverseRodriguezFormula<double>(topocentric2ecef.transpose()*
                                                                                             StereoVision::Geometry::rodriguezFormula<double>(body2ecef.r)); //body2topocentric
#ifndef NDEBUG
            ecefOrientCache.push_back(StereoVisionApp::Trajectory::TimeCartesianBlock{.time=currentT,.val=body2ecef.r});
#endif
            orientCache.push_back(StereoVisionApp::Trajectory::TimeCartesianBlock{.time=currentT,.val=orient});

            Eigen::Vector3d gps = staticTrajSimulator.gps(currentT);
            //Eigen::Vector3d gpsVelocity = staticTrajSimulator.gpsVelocity(currentT);

            gpsCache.position->push_back(StereoVisionApp::Trajectory::TimeCartesianBlock{.time=currentT,.val=gps});
            //gpsCache.velocities->push_back(StereoVisionApp::Trajectory::TimeCartesianBlock{.time=currentT,.val=gpsVelocity});
        }
        Eigen::Vector3d gyro = staticTrajSimulator.gyro(currentT);
        Eigen::Vector3d acc = staticTrajSimulator.acc(currentT);

        gyroCache.push_back(StereoVisionApp::Trajectory::TimeCartesianBlock{.time=currentT,.val=gyro});
        accCache.push_back(StereoVisionApp::Trajectory::TimeCartesianBlock{.time=currentT,.val=acc});

        currentT += dt;
        tickCount++;
    }

    genTraj->setCachedRawPos(posCache);
    genTraj->setCachedRawOrient(orientCache);
    genTraj->setCachedRawGps(gpsCache);
    genTraj->setCachedRawGyro(gyroCache);
    genTraj->setCachedRawAcc(accCache);

    traj->setOrientationAngleRepresentation(StereoVisionApp::Trajectory::AxisAngle);
    traj->setOrientationAngleUnits(StereoVisionApp::Trajectory::Radians);
    traj->setOrientationTopocentricConvention(topocentricConvention);

    traj->setPreIntegrationTime(0.5);
    traj->setGpsAccuracy(0.02);
    traj->setGyroAccuracy(0.1);
    traj->setAccAccuracy(0.5);
    traj->setFixed(true);

    //setup cameras
    std::vector<StereoVision::Geometry::RigidBodyTransform<float>> cameras2body =
    {
        StereoVision::Geometry::RigidBodyTransform<float>(Eigen::Vector3f(0.0001929,-0.0000297,0.000113), Eigen::Vector3f(-0.00276, 0.00223, 1.00175)),
        StereoVision::Geometry::RigidBodyTransform<float>(Eigen::Vector3f(-0.0001529,-0.1000193,0.000161), Eigen::Vector3f(-0.50201, -0.000927, 0.79826)),
        StereoVision::Geometry::RigidBodyTransform<float>(Eigen::Vector3f(0.0001729,0.10000313,-0.000123), Eigen::Vector3f( 0.50192, -0.00183, 0.79267)),
    };
    std::vector<double> camTimes = {5, 15, 25, 45, 55, 65};

    qint64 camera1Id = project.createDataBlock(StereoVisionApp::Camera::staticMetaObject.className());
    qint64 camera2Id = project.createDataBlock(StereoVisionApp::Camera::staticMetaObject.className());
    qint64 camera3Id = project.createDataBlock(StereoVisionApp::Camera::staticMetaObject.className());

    qint64 mounting1Id = project.createDataBlock(StereoVisionApp::Mounting::staticMetaObject.className());
    qint64 mounting2Id = project.createDataBlock(StereoVisionApp::Mounting::staticMetaObject.className());
    qint64 mounting3Id = project.createDataBlock(StereoVisionApp::Mounting::staticMetaObject.className());

    StereoVisionApp::Camera* cam1 = project.getDataBlock<StereoVisionApp::Camera>(camera1Id);
    StereoVisionApp::Camera* cam2 = project.getDataBlock<StereoVisionApp::Camera>(camera2Id);
    StereoVisionApp::Camera* cam3 = project.getDataBlock<StereoVisionApp::Camera>(camera3Id);

    StereoVisionApp::Mounting* mounting1 = project.getDataBlock<StereoVisionApp::Mounting>(mounting1Id);
    StereoVisionApp::Mounting* mounting2 = project.getDataBlock<StereoVisionApp::Mounting>(mounting2Id);
    StereoVisionApp::Mounting* mounting3 = project.getDataBlock<StereoVisionApp::Mounting>(mounting3Id);

    QVERIFY(cam1 != nullptr);
    QVERIFY(cam2 != nullptr);
    QVERIFY(cam3 != nullptr);

    QVERIFY(mounting1 != nullptr);
    QVERIFY(mounting2 != nullptr);
    QVERIFY(mounting3 != nullptr);

    std::array<StereoVisionApp::Camera*, 3> cams{cam1, cam2, cam3};

    std::array<StereoVisionApp::Mounting*, 3> mountings{mounting1, mounting2, mounting3};

    constexpr int imgWidth = 640;
    constexpr int imgHeight = 480;

    constexpr float imgFLen = 200.;
    constexpr float imgPPx = 640./2;
    constexpr float imgPPy = 480./2;

    StereoVisionApp::floatParameter fLen(imgFLen);
    StereoVisionApp::floatParameter ppX(imgPPx);
    StereoVisionApp::floatParameter ppY(imgPPy);

    fLen.setUncertainty(10.);
    ppX.setUncertainty(10.);
    ppY.setUncertainty(10.);

    StereoVisionApp::floatParameter zero(0);
    zero.setUncertainty(1.0); //prior for zero

    for (StereoVisionApp::Camera* cam : cams) {
        cam->setImHeight(imgHeight);
        cam->setImWidth(imgWidth);

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
    }

    QCOMPARE(mountings.size(), cameras2body.size());

    for (size_t i = 0; i < cameras2body.size(); i++) {

        StereoVisionApp::Mounting* mounting = mountings[i];

        StereoVision::Geometry::RigidBodyTransform<float> const& camera2body = cameras2body[i];
        StereoVision::Geometry::RigidBodyTransform<float> camera2bodyInitial(Eigen::Vector3f::Zero(),Eigen::Vector3f::Zero());

        mounting->setOptTransform(camera2body);
    }

    //build images and correspondances

    qint64 correspSetId = project.createDataBlock(StereoVisionApp::CorrespondencesSet::staticMetaObject.className());
    StereoVisionApp::CorrespondencesSet* correspSet = project.getDataBlock<StereoVisionApp::CorrespondencesSet>(correspSetId);

    QVERIFY(correspSet != nullptr);

    correspSet->setVerbose(true);

    std::vector<std::array<qint64,3>> imagesIdxs;
    std::vector<std::array<StereoVision::Geometry::RigidBodyTransform<float>,3>> gtImages2Mapping;
    imagesIdxs.reserve(camTimes.size());
    gtImages2Mapping.reserve(camTimes.size());

    std::vector<qint64> lmsIdxs;
    std::vector<Eigen::Vector3f> lmsLocalPos;
#ifndef NDEBUG
    std::vector<Eigen::Vector3f> lmsCamPos;
    std::vector<Eigen::Vector3f> lmsCamNormPos;
    std::vector<Eigen::Vector3f> lmsCamImgPos;
#endif

    int nMatches = 0;

    std::uniform_real_distribution<double> uDist(0,imgWidth);
    std::uniform_real_distribution<double> vDist(0,imgHeight);
    std::uniform_real_distribution<double> lDist(0,1);
    std::uniform_real_distribution<float> lmDist(-50,50);

    auto buildImagesCorrespondances = [correspSet, this, &uDist, &vDist, &lDist, &nMatches, imgFLen]
        (
            qint64 img1_id,
            StereoVision::Geometry::RigidBodyTransform<float> const& img12mappingGt,
            qint64 img2_id,
            StereoVision::Geometry::RigidBodyTransform<float> const& img22mappingGt,
            int nPts) {

            constexpr StereoVisionApp::Correspondences::Types UV = StereoVisionApp::Correspondences::UV;
            using UVCorresp = StereoVisionApp::Correspondences::Typed<UV>;

            StereoVision::Geometry::AffineTransform<float> img12img2 = (img22mappingGt.inverse()*img12mappingGt).toAffineTransform();

            Eigen::Matrix3d E = StereoVision::Geometry::transform2EssentialMatrix(img12img2.cast<double>());
            Eigen::Matrix3d Kinv =  Eigen::Matrix3d::Identity();
            Kinv(0,0) = 1/imgFLen;
            Kinv(1,1) = 1/imgFLen;
            Kinv(0,2) = -imgPPx/imgFLen;
            Kinv(1,2) = -imgPPy/imgFLen;

            Eigen::Matrix3d F = Kinv.transpose()*E*Kinv;

            for (int i = 0; i < nPts; i++) {

                Eigen::Vector3d uv1(uDist(_re), vDist(_re), 1);

                Eigen::Vector3d l = F*uv1;

                double u20;
                double v20;

                double u2f;
                double v2f;

                if (std::abs(l[0]) < 1e-6 and std::abs(l[1]) < 1e-6) {
                    continue; //skip the point
                } else if (std::abs(l[0]) < 1e-6) {
                    u20 = 0;
                    v20 = -l[2]/l[1];

                    u2f = imgWidth;
                    v2f = -(l[0]*imgWidth+l[2])/l[1];
                } else {
                    v20 = 0;
                    u20 = -l[2]/l[0];

                    v2f = imgHeight;
                    u2f = -(l[1]*imgHeight+l[2])/l[0];
                }

                if (u20 < 0) {
                    u20 = 0;
                    v20 = -l[2]/l[1];
                } else if (u20 > imgWidth) {
                    u20 = imgWidth;
                    v20 = -(l[0]*imgWidth+l[2])/l[1];
                }
                if (u2f > imgWidth) {
                    u2f = imgWidth;
                    v2f = -(l[0]*imgWidth+l[2])/l[1];
                } else if (u2f < 0) {
                    u2f = 0;
                    v2f = -l[2]/l[1];
                }

                if (v20 < 0) {
                    v20 = 0;
                    u20 = -l[2]/l[0];
                } else if (v20 > imgHeight) {
                    v20 = imgHeight;
                    u20 = -(l[1]*imgHeight+l[2])/l[0];
                }
                if (v2f > imgHeight) {
                    v2f = imgHeight;
                    u2f = -(l[1]*imgHeight+l[2])/l[0];
                } else if (v2f < 0) {
                    v2f = 0;
                    u2f = -l[2]/l[0];
                }

                if (u20 < 0 or u20 > imgWidth) {
                    continue;
                }
                if (u2f > imgWidth or u2f < 0) {
                    continue;
                }
                if (v20 < 0 or v20 > imgHeight) {
                    continue;
                }
                if (v2f > imgHeight or v2f < 0) {
                    continue;
                }

                double du = u2f - u20;
                double dv = v2f - v20;

                double rate = lDist(_re);

                Eigen::Vector2d uv2(u20+rate*du, v20+rate*dv);

                #ifndef NDEBUG
                do {
                    Eigen::Vector3f ptCam1(uv1.x(), uv1.y(), 1);
                    ptCam1.z() = imgFLen;

                    Eigen::Vector3f ptCam1Normalized = ptCam1;
                    ptCam1Normalized.x() -= imgPPx;
                    ptCam1Normalized.y() -= imgPPy;

                    Eigen::Vector3f ptCam2 = img12img2*ptCam1Normalized;
                    ptCam2 /= ptCam2.z();

                    ptCam2 *= imgFLen;
                    ptCam2.x() += imgPPx;
                    ptCam2.y() += imgPPy;
                    ptCam2.z() = 1;

                    float resEpipolar = ptCam2.dot(l.cast<float>());

                    if (std::abs(resEpipolar)/std::max<float>(1,l.norm()) > 1e-4) {
                        break;
                    }
                    QVERIFY(std::abs(resEpipolar)/std::max<float>(1,l.norm()) < 1e-4);
                } while(0);
                #endif

                UVCorresp uvCorresp1;
                uvCorresp1.blockId = img1_id;
                uvCorresp1.u = uv1.x();
                uvCorresp1.v = uv1.y();
                uvCorresp1.sigmaU = 1;
                uvCorresp1.sigmaV = 1;

                UVCorresp uvCorresp2;
                uvCorresp2.blockId = img2_id;
                uvCorresp2.u = uv2.x();
                uvCorresp2.v = uv2.y();
                uvCorresp2.sigmaU = 1;
                uvCorresp2.sigmaV = 1;

                correspSet->addCorrespondence({uvCorresp1, uvCorresp2});
                nMatches++;
            }

        };

    auto buildImage2LandmarkCorrespondance = [correspSet, &nMatches] (qint64 img_id,
                                                                StereoVision::Geometry::RigidBodyTransform<float> const& img2mappingGt,
                                                                qint64 lm_id,
                                                                Eigen::Vector3f const& lmPosMapping) {

        constexpr StereoVisionApp::Correspondences::Types ID = StereoVisionApp::Correspondences::PRIORID;
        constexpr StereoVisionApp::Correspondences::Types UV = StereoVisionApp::Correspondences::UV;
        using IDCorresp = StereoVisionApp::Correspondences::Typed<ID>;
        using UVCorresp = StereoVisionApp::Correspondences::Typed<UV>;

        StereoVision::Geometry::RigidBodyTransform<float> mapping2img = img2mappingGt.inverse();
        Eigen::Vector3f lmCam = mapping2img*lmPosMapping;

        float uPos = lmCam[0]/lmCam[2] * imgFLen;
        float vPos = lmCam[1]/lmCam[2] * imgFLen;

        uPos += imgPPx;
        vPos += imgPPy;

        if (uPos < 0 or uPos > imgWidth) {
            return;
        }

        if (vPos < 0 or vPos > imgHeight) {
            return;
        }

        IDCorresp idCorresp;
        idCorresp.blockId = lm_id;

        UVCorresp uvCorresp;
        uvCorresp.blockId = img_id;
        uvCorresp.u = uPos;
        uvCorresp.v = vPos;
        uvCorresp.sigmaU = 1;
        uvCorresp.sigmaV = 1;

        correspSet->addCorrespondence({idCorresp, uvCorresp});
        nMatches++;

    };

    for (size_t i = 0; i < camTimes.size(); i++) {
        double t = camTimes[i];

        ceres::Jet<ceres::Jet<double,1>,1> jet_t;
        jet_t.a.a = t;
        jet_t.a.v[0] = 0;
        jet_t.v[0].a = 0;
        jet_t.v[0].v[0] = 0;

        auto body2WorldJet = trajFunctor->trajectory(jet_t);

        StereoVision::Geometry::RigidBodyTransform<double> body2world(
            Eigen::Vector3d(body2WorldJet.r[0].a.a, body2WorldJet.r[1].a.a, body2WorldJet.r[2].a.a),
            Eigen::Vector3d(body2WorldJet.t[0].a.a, body2WorldJet.t[1].a.a, body2WorldJet.t[2].a.a));

        StereoVision::Geometry::RigidBodyTransform<float> body2mapping = (ecef2LocalRigid*body2world).cast<float>();

        QString imgNameTemplate = QString("im%2_t%1").arg(t);

        qint64 image1Id = project.createDataBlock(StereoVisionApp::Image::staticMetaObject.className());
        qint64 image2Id = project.createDataBlock(StereoVisionApp::Image::staticMetaObject.className());
        qint64 image3Id = project.createDataBlock(StereoVisionApp::Image::staticMetaObject.className());

        StereoVisionApp::Image* img1 = project.getDataBlock<StereoVisionApp::Image>(image1Id);
        StereoVisionApp::Image* img2 = project.getDataBlock<StereoVisionApp::Image>(image2Id);
        StereoVisionApp::Image* img3 = project.getDataBlock<StereoVisionApp::Image>(image3Id);

        img1->setObjectName(imgNameTemplate.arg(1));
        img2->setObjectName(imgNameTemplate.arg(2));
        img3->setObjectName(imgNameTemplate.arg(3));

        img1->assignTrajectory(trajectoryId);
        img2->assignTrajectory(trajectoryId);
        img3->assignTrajectory(trajectoryId);

        img1->assignCamera(camera1Id);
        img2->assignCamera(camera2Id);
        img3->assignCamera(camera3Id);

        img1->assignMounting(mounting1Id);
        img2->assignMounting(mounting2Id);
        img3->assignMounting(mounting3Id);

        img1->setImageTimeStamp(t);
        img2->setImageTimeStamp(t);
        img3->setImageTimeStamp(t);

        std::array<StereoVisionApp::Image*,3> images{img1, img2, img3};

        imagesIdxs.emplace_back(std::array<qint64,3>{image1Id, image2Id, image3Id});

        std::array<StereoVision::Geometry::RigidBodyTransform<float>,3> poses;

        for (int j = 0; j < 3; j++) {
            StereoVision::Geometry::RigidBodyTransform<float> const& camera2body = cameras2body[j];
            poses[j] = body2mapping*camera2body;
            images[j]->setOptTransform(poses[j]);
        }

        gtImages2Mapping.emplace_back(std::move(poses));

        //for all three current images build image to image matches
        for (int c = 0; c < 3; c++) {

            qint64 img1_id = imagesIdxs[i][c];
            StereoVision::Geometry::RigidBodyTransform<float> const& img12mappingGt = gtImages2Mapping[i][c];

            //go back to previous images
            for (size_t j = 0; j < i; j++) {
                //one by one
                for (int k = 0; k < 3; k++) {

                    qint64 img2_id = imagesIdxs[j][k];
                    StereoVision::Geometry::RigidBodyTransform<float> const& img22mappingGt = gtImages2Mapping[j][k];

                    constexpr int nPts = 8; //not too many points to not delay the optimizer too much
                    buildImagesCorrespondances(img1_id, img12mappingGt, img2_id, img22mappingGt, nPts);
                }
            }

            //match with the images on the rig as well
            for (int k = 0; k < c; k++) {

                qint64 img2_id = imagesIdxs[i][k];
                StereoVision::Geometry::RigidBodyTransform<float> const& img22mappingGt = gtImages2Mapping[i][k];

                constexpr int nPts = 5; //not too many points to not delay the optimizer too much
                buildImagesCorrespondances(img1_id, img12mappingGt, img2_id, img22mappingGt, nPts);
            }
        }

        //add previous landmark to current image (when seen)

        for (size_t j = 0; j <  lmsIdxs.size(); j++) {

            qint64 lmId = lmsIdxs[j];
            Eigen::Vector3f lmMappingPos = lmsLocalPos[j];

            //for all three current images build image to image matches
            for (int c = 0; c < 3; c++) {

                qint64 img_id = imagesIdxs[i][c];
                StereoVision::Geometry::RigidBodyTransform<float> const& img2mappingGt = gtImages2Mapping[i][c];

                buildImage2LandmarkCorrespondance(img_id, img2mappingGt, lmId, lmMappingPos);
            }
        }

        //build the landmark and add it to all previous image (when seen)
        std::array<float, 3> viewDists = {350, 500, 700};

        for (float vDist : viewDists) {
            Eigen::Vector3f camPos = Eigen::Vector3f(0 + lmDist(_re),0 + lmDist(_re), vDist + lmDist(_re));
            Eigen::Vector3f lmMappingPos = poses[0] * camPos;
            Eigen::Vector3d lmEcefPos = local2Ecef*lmMappingPos.cast<double>();
            #ifndef NDEBUG
            lmsCamPos.push_back(camPos);
            lmsCamNormPos.push_back(camPos/camPos[2]);
            lmsCamImgPos.push_back(camPos/camPos[2] * imgFLen + Eigen::Vector3f(imgPPx, imgPPy, 0));
            #endif

            qint64 lmId = project.createDataBlock(StereoVisionApp::Landmark::staticMetaObject.className());

            StereoVisionApp::Landmark* lm = project.getDataBlock<StereoVisionApp::Landmark>(lmId);

            if (lm == nullptr) {
                continue;
            }

            double stdDevPos = 0.1;
            StereoVisionApp::floatParameter x_p(lmEcefPos[0], stdDevPos);
            StereoVisionApp::floatParameter y_p(lmEcefPos[1], stdDevPos);
            StereoVisionApp::floatParameter z_p(lmEcefPos[2], stdDevPos);

            lm->setXCoord(x_p);
            lm->setYCoord(y_p);
            lm->setZCoord(z_p);

            lmsIdxs.push_back(lmId);
            lmsLocalPos.push_back(lmMappingPos);

            //go back to previous images
            for (size_t j = 0; j <= i; j++) {
                //cameras
                for (int k = 0; k < 3; k++) {

                    qint64 img_id = imagesIdxs[j][k];
                    StereoVision::Geometry::RigidBodyTransform<float> const& img2mappingGt = gtImages2Mapping[j][k];

                    buildImage2LandmarkCorrespondance(img_id, img2mappingGt, lmId, lmMappingPos);
                }
            }
        }


    }

    int nLandmarks = lmsIdxs.size();
    int nImages = imagesIdxs.size()*3;
    qInfo() << "Number of images: " << nImages << " Number of landmarks: " << nLandmarks << " Number of correspondences: " << nMatches;

    //run the optimizer
    constexpr bool computeUncertainty = false;

    StereoVisionApp::ModularSBASolver sbaSolver(&project, computeUncertainty);
    sbaSolver.setSilent(true);
    sbaSolver.setFuncTolerance(1e-8);
    sbaSolver.setParamsTolerance(1e-10);
    sbaSolver.setOptimizationSteps(100);

    sbaSolver.enableLogging(QDir::currentPath(), "cams2traj_test_");

    StereoVisionApp::TrajectoryBaseSBAModule* trajSBAModule =
        new StereoVisionApp::TrajectoryBaseSBAModule(traj->getPreIntegrationTime());
    bool trajModuleAdded = sbaSolver.addModule(trajSBAModule);

    bool imModuleAdded = sbaSolver.addModule(new StereoVisionApp::ImageAlignementSBAModule());
    bool lmModuleAdded = sbaSolver.addModule(new StereoVisionApp::LandmarksSBAModule());
    bool mntModuleAdded = sbaSolver.addModule(new StereoVisionApp::MountingsSBAModule());
    bool correspModuleAdded = sbaSolver.addModule(new StereoVisionApp::CorrespondencesSetSBAModule());


    QVERIFY(trajModuleAdded);
    QVERIFY(imModuleAdded);
    QVERIFY(lmModuleAdded);
    QVERIFY(mntModuleAdded);
    QVERIFY(correspModuleAdded);

    bool initSuccess = sbaSolver.init();

    QVERIFY(initSuccess);

    QVERIFY(sbaSolver.itemIsObservable(trajectoryId));

    // ensure the mountings have been added to the factor graph.
    QVERIFY(sbaSolver.getPoseNode(mounting1Id) != nullptr);
    QVERIFY(sbaSolver.getPoseNode(mounting2Id) != nullptr);
    QVERIFY(sbaSolver.getPoseNode(mounting3Id) != nullptr);

    ceres::Problem* problem = sbaSolver.ceresProblem();

    QVERIFY(problem != nullptr);

    //check something was added in the factor graph
    QVERIFY(problem->NumResidualBlocks() > 0);


    bool optSuccess = sbaSolver.opt_step();

    QVERIFY(optSuccess);

    bool stdSuccess = sbaSolver.std_step();

    QVERIFY(stdSuccess);

    bool writeOk = sbaSolver.writeResults();

    QVERIFY(writeOk);

    bool writeStdOk = sbaSolver.writeUncertainty();

    QVERIFY(writeStdOk);

    sbaSolver.cleanup();

    StereoVisionApp::StatusOptionalReturn<StereoVisionApp::Trajectory::TimeTrajectorySequence> optoptTraj =
        traj->optimizedTrajectory();

    QVERIFY(optoptTraj.isValid());

    for (size_t i = 0; i < mountings.size(); i++) {

        QVERIFY(mountings[i]->optPos().isSet());
        QVERIFY(mountings[i]->optRot().isSet());

        StereoVision::Geometry::RigidBodyTransform<float> const& gt = cameras2body[i];

        auto optTransform = mountings[i]->getOptRigidTransform();

        QVERIFY(optTransform.has_value());

        StereoVision::Geometry::RigidBodyTransform<float> const& transform = optTransform.value();

        StereoVision::Geometry::RigidBodyTransform<float> delta = gt.inverse()*transform;
        delta.t = transform.t - gt.t;

        double errR = delta.r.norm();
        double errT = delta.t.norm();

        if (errR >= 1e-5 or errT >= 1e-3) {
            qWarning() << "Error for mounting: " << (i+1);
        }
        if (errR >= 1e-5) {
            qWarning()<< "transform.r = [" << transform.r[0] << " " << transform.r[1] << " " << transform.r[2] << "]"
                       << " gt.r = [" << gt.r[0] << " " << gt.r[1] << " " << gt.r[2] << "]"
                       << " errR = " << errR;
        }
        if (errT >= 1e-3) {
            qWarning()<< "transform.t = [" << transform.t[0] << " " << transform.t[1] << " " << transform.t[2] << "]"
                       << " gt.t = [" << gt.t[0] << " " << gt.t[1] << " " << gt.t[2] << "]"
                       << "errT = " << errT;
        }

        QVERIFY(errR < 1e-5);
        QVERIFY(errT < 1e-3);
    }

}

QTEST_MAIN(TestSBASolver);
#include "main.moc"
