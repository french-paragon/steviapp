#include <QtTest/QtTest>

#include <time.h>


#include "../../libs/datablocks/project.h"
#include "../../libs/datablocks/image.h"
#include "../../libs/datablocks/camera.h"
#include "../../libs/datablocks/landmark.h"
#include "../../libs/datablocks/correspondencesset.h"
#include "../../libs/datablocks/localcoordinatesystem.h"

#include "../../libs/sparsesolver/modularsbasolver.h"
#include "../../libs/sparsesolver/sbamodules/landmarkssbamodule.h"
#include "../../libs/sparsesolver/sbamodules/pinholecameraprojectormodule.h"
#include "../../libs/sparsesolver/sbamodules/imagealignementsbamodule.h"
#include "../../libs/sparsesolver/sbamodules/correspondencessetsbamodule.h"
#include "../../libs/sparsesolver/sbamodules/localcoordinatesystemsbamodule.h"


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

protected:

    void genericPnPTest(bool withCorrespondance, bool fixedPoints);

};

void TestSBASolver::genericPnPTest(bool withCorrespondance, bool fixedPoints) {

    StereoVisionApp::ProjectFactory& pF = StereoVisionApp::ProjectFactory::defaultProjectFactory();

    StereoVisionApp::Project* pPtr = pF.createProject(this);

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

    delete pPtr;

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

    StereoVisionApp::Project* pPtr = pF.createProject(this);

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

    delete pPtr;

}


void TestSBASolver::image2RigidBody() {

    StereoVisionApp::ProjectFactory& pF = StereoVisionApp::ProjectFactory::defaultProjectFactory();

    StereoVisionApp::Project* pPtr = pF.createProject(this);

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

    delete pPtr;

}
void TestSBASolver::rigidBody2RigidBody() {

    StereoVisionApp::ProjectFactory& pF = StereoVisionApp::ProjectFactory::defaultProjectFactory();

    StereoVisionApp::Project* pPtr = pF.createProject(this);

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

    delete pPtr;

}


QTEST_MAIN(TestSBASolver);
#include "main.moc"
