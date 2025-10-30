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

    void basicTrajectory();
    void movingBody2MovingBody();

    void testWithEarthGravity();

protected:

    void genericPnPTest(bool withCorrespondance, bool fixedPoints);
    void configureStandardTrajectory(StereoVisionApp::Trajectory* traj, QString const& sbet, QString const& ins);

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
    traj->setGyroSign(StereoVisionApp::Trajectory::Axis::X, 1);
    traj->setGyroSign(StereoVisionApp::Trajectory::Axis::Y, 1);
    traj->setGyroSign(StereoVisionApp::Trajectory::Axis::Z, 1);

    traj->setPreIntegrationTime(0.1);
    traj->setGpsAccuracy(0.02);
    traj->setGyroAccuracy(0.1);
    traj->setAccAccuracy(0.5);
}

void TestSBASolver::initTestCase() {

    srand(time(nullptr));

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

void TestSBASolver::basicTrajectory() {

    StereoVisionApp::ProjectFactory& pF = StereoVisionApp::ProjectFactory::defaultProjectFactory();

    std::unique_ptr<StereoVisionApp::Project> pPtr(pF.createProject(this)); //use unique ptr to ensure project is deleted at the end of test case

    QVERIFY(pPtr != nullptr);

    StereoVisionApp::Project& project = *pPtr;

    qint64 trajectoryId = project.createDataBlock(StereoVisionApp::Trajectory::staticMetaObject.className());

    StereoVisionApp::Trajectory* traj = project.getDataBlock<StereoVisionApp::Trajectory>(trajectoryId);

    QVERIFY(traj != nullptr);

    configureStandardTrajectory(traj, ":/trajectories/zero_acc/sbet.csv", ":/trajectories/zero_acc/ins.csv");

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

    StereoVision::Geometry::RigidBodyTransform<double> body2sensor =
        StereoVision::Geometry::RigidBodyTransform<double>(optMountingTransform.value().cast<double>());

    StereoVision::Geometry::RigidBodyTransform<double> sensor2body = body2sensor.inverse();

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
            interpolableBody1ToMapping.weigthLower,
            interpolableBody1ToMapping.valLower,
            interpolableBody1ToMapping.weigthUpper,
            interpolableBody1ToMapping.valUpper);

        StereoVision::Geometry::RigidBodyTransform<double> sensor1ToMapping =
            body1ToMapping*sensor2body;

        auto interpolableBody2ToMapping = trajData.getValueAtTime(times[1]);
        StereoVision::Geometry::RigidBodyTransform<double> body2ToMapping =
            StereoVision::Geometry::interpolateRigidBodyTransformOnManifold(
                interpolableBody2ToMapping.weigthLower,
                interpolableBody2ToMapping.valLower,
                interpolableBody2ToMapping.weigthUpper,
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


    StereoVisionApp::ModularSBASolver sbaSolver(&project);
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

void TestSBASolver::testWithEarthGravity() {

    StereoVisionApp::ProjectFactory& pF = StereoVisionApp::ProjectFactory::defaultProjectFactory();

    StereoVisionApp::Project* pPtr = pF.createProject(this);

    QVERIFY(pPtr != nullptr);

    StereoVisionApp::Project& project = *pPtr;

    project.setDefaultProjectCRS("EPSG:4978");

    constexpr double EarthRadius = 6357000;

    StereoVision::Geometry::AffineTransform<double> ecef2Local(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0,0,-EarthRadius));

    project.setLocalCoordinateFrame(ecef2Local);

    qint64 trajectoryId = project.createDataBlock(StereoVisionApp::Trajectory::staticMetaObject.className());

    StereoVisionApp::Trajectory* traj = project.getDataBlock<StereoVisionApp::Trajectory>(trajectoryId);

    QVERIFY(traj != nullptr);

    StereoVisionApp::GeneratedTrajectory* genTraj = qobject_cast<StereoVisionApp::GeneratedTrajectory*>(traj);

    QVERIFY(genTraj != nullptr);

    using TrajGeneratorInfos = StereoVisionApp::GeneratedTrajectory::TrajGeneratorInfos;

    constexpr double t0 = 0;
    constexpr double tf = 100;
    constexpr double dt = tf-t0;

    constexpr double rotationRate = 2*M_PI/(tf-t0);
    constexpr double circleRadius = 1000;

    constexpr double gAmpl = 9.81;

    double dtPos = 0.5;
    double dtAcc = 0.1;

    struct PositionCalculator {
        static Eigen::Vector3d pos(double t) {
            double x = circleRadius*std::cos(t/dt * 2*M_PI);
            double y = circleRadius*std::sin(t/dt * 2*M_PI);
            double z = EarthRadius;
            return Eigen::Vector3d(x,y,z);
        }
    };

    genTraj->setInitialAndFinalTimes(t0,tf);

    genTraj->setAngularSpeedGenerator(TrajGeneratorInfos{[] (double t) {
                                                             double omega = rotationRate;
                                                             (void) t; return Eigen::Vector3d(0,0,omega);
                                                         }, dtAcc}); //constant rotation rate
    genTraj->setAccelerationGenerator(TrajGeneratorInfos{[&ecef2Local, gAmpl] (double t) {
                                                             double acc = -circleRadius*rotationRate*rotationRate;
                                                             Eigen::Vector3d pos = ecef2Local*PositionCalculator::pos(t);
                                                             Eigen::Vector3d g = ecef2Local.t - pos;
                                                             g.normalize();
                                                             g *= gAmpl; //g keep its amplitude and points toward center of the earth
                                                             Eigen::Vector3d ret = Eigen::Vector3d(0,acc,0) - g;
                                                             return ret;
                                                         }, dtAcc});

    genTraj->setPositionGenerator(TrajGeneratorInfos{[] (double t) {
                                                         return PositionCalculator::pos(t);
                                                     }, dtPos});
    genTraj->setOrientationGenerator(TrajGeneratorInfos{[] (double t) {
                                                            double rz = t/dt * 2*M_PI - M_PI_2;
                                                            return Eigen::Vector3d(0,0,rz);
                                                        }, dtPos});



    traj->setPreIntegrationTime(0.5);
    traj->setGpsAccuracy(0.02);
    traj->setGyroAccuracy(0.1);
    traj->setAccAccuracy(0.5);



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

    for (int i = 0; i < optTraj.nPoints(); i++) {
        auto& node = optTraj[i];
        Eigen::Vector3d expected = ecef2Local*PositionCalculator::pos(optTraj[i].time);

        double delta = (node.val.t - expected).norm() / expected.norm();

        double threshold = 1e-6;

        if (delta >= threshold) {
            qWarning() << "About to fail with delta = " << delta << " (t = " << node.time << " position = "
                    << node.val.t.x() << " " << node.val.t.y() << " " << node.val.t.z()
                    << ", expected = " << expected.x() << " " << expected.y() << " " << expected.z() << ")";
        }

        QVERIFY(delta < threshold);
    }

}

QTEST_MAIN(TestSBASolver);
#include "main.moc"
