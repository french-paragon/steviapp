#include <QtTest/QtTest>

#include <time.h>

#include <StereoVision/geometry/rotations.h>

#include "../../libs/datablocks/project.h"
#include "../../libs/datablocks/camera.h"
#include "../../libs/datablocks/cameras/pushbroompinholecamera.h"

#include "../../libs/sparsesolver/modularsbasolver.h"
#include "../../libs/sparsesolver/sbamodules/pinholecameraprojectormodule.h"
#include "../../libs/sparsesolver/costfunctors/modularuvprojection.h"

class TestUvProjector : public QObject
{
    Q_OBJECT
public:

private Q_SLOTS:

    void initTestCase();

    void testPinholeUVProjector();
    void testPushBroomPinholeUVProjector();
    void testCrossUVProjector();

};

void TestUvProjector::initTestCase() {

    srand(time(nullptr));

}

void TestUvProjector::testPinholeUVProjector() {

    StereoVisionApp::Project project(nullptr);

    StereoVisionApp::ModularSBASolver sbaSolver(&project);

    ceres::Problem problem;

    StereoVisionApp::Camera camera(&project);

    camera.setImHeight(480);
    camera.setImWidth(640);

    StereoVisionApp::floatParameter fLen(35);
    StereoVisionApp::floatParameter ppX(pFloatType(camera.imWidth())/2);
    StereoVisionApp::floatParameter ppY(pFloatType(camera.imHeight())/2);

    StereoVisionApp::floatParameter zero(0);

    camera.setFLen(fLen);
    camera.setOpticalCenterX(ppX);
    camera.setOpticalCenterY(ppY);

    camera.setB1(zero);
    camera.setB2(zero);

    camera.setP1(zero);
    camera.setP2(zero);

    camera.setK1(zero);
    camera.setK2(zero);
    camera.setK3(zero);
    camera.setK4(zero);
    camera.setK5(zero);
    camera.setK6(zero);

    StereoVisionApp::PinholeCamProjModule projectorModule(&camera);

    projectorModule.setVerbose(false);
    projectorModule.setup(&sbaSolver, problem);

    bool initOk = projectorModule.init();

    QVERIFY(initOk);

    std::array<double, 3> pointPos = {42,33,fLen.value()};
    std::array<double, 3> camRot = {0,0,0};
    std::array<double, 3> camPos = {0,0,0};

    Eigen::Vector2d ptPos;
    ptPos << pointPos[0] + ppX.value(), pointPos[1] + ppY.value();
    ptPos /= pointPos[2];
    ptPos *= fLen.value();

    Eigen::Matrix2d stiffness = Eigen::Matrix2d::Identity();

    bool addingOk = projectorModule.addProjectionCostFunction(pointPos.data(), camRot.data(), camPos.data(), ptPos, stiffness);

    QVERIFY(addingOk);

    //check the cost was effectively added
    QVERIFY(problem.NumResidualBlocks() >= 1);

    double cost;

    ceres::Problem::EvaluateOptions options;

    bool evaluateOk = problem.Evaluate(options, &cost, nullptr, nullptr, nullptr);

    QVERIFY(evaluateOk);

    QVERIFY(std::abs(cost) < 1e-6);

}
void TestUvProjector::testPushBroomPinholeUVProjector() {

    StereoVisionApp::Project project(nullptr);

    StereoVisionApp::ModularSBASolver sbaSolver(&project);

    ceres::Problem problem;

    StereoVisionApp::PushBroomPinholeCamera camera(&project);

    camera.setImWidth(640);

    StereoVisionApp::floatParameter fLen(35);
    StereoVisionApp::floatParameter ppX(pFloatType(camera.imWidth())/2);

    StereoVisionApp::floatParameter zero(0);

    camera.setFLen(fLen);
    camera.setOpticalCenterX(ppX);

    camera.setA1(zero);
    camera.setA2(zero);
    camera.setA3(zero);
    camera.setA4(zero);
    camera.setA5(zero);

    camera.setB1(zero);
    camera.setB2(zero);
    camera.setB3(zero);
    camera.setB4(zero);
    camera.setB5(zero);

    StereoVisionApp::PinholePushBroomCamProjectorModule projectorModule(&camera);

    projectorModule.setVerbose(false);
    projectorModule.setup(&sbaSolver, problem);

    bool initOk = projectorModule.init();

    QVERIFY(initOk);

    std::array<double, 3> pointPos = {42,0,fLen.value()};
    std::array<double, 3> camRot = {0,0,0};
    std::array<double, 3> camPos = {0,0,0};

    Eigen::Vector2d ptPos;
    ptPos << pointPos[0] + ppX.value(), pointPos[1];
    ptPos /= pointPos[2];
    ptPos *= fLen.value();

    Eigen::Matrix2d stiffness = Eigen::Matrix2d::Identity();

    bool addingOk = projectorModule.addProjectionCostFunction(pointPos.data(), camRot.data(), camPos.data(), ptPos, stiffness);

    QVERIFY(addingOk);

    //check the cost was effectively added
    QVERIFY(problem.NumResidualBlocks() >= 1);

    double cost;

    ceres::Problem::EvaluateOptions options;

    bool evaluateOk = problem.Evaluate(options, &cost, nullptr, nullptr, nullptr);

    QVERIFY(evaluateOk);

    QVERIFY(std::abs(cost) < 1e-6);

}
void TestUvProjector::testCrossUVProjector() {

    StereoVisionApp::Project project(nullptr);

    StereoVisionApp::ModularSBASolver sbaSolver(&project);

    ceres::Problem problem;

    StereoVisionApp::Camera ph_camera(&project);
    StereoVisionApp::PushBroomPinholeCamera pb_camera(&project);

    pb_camera.setImWidth(640);

    StereoVisionApp::floatParameter fLen(35);

    StereoVisionApp::floatParameter ph_ppX(pFloatType(ph_camera.imWidth())/2);
    StereoVisionApp::floatParameter ph_ppY(pFloatType(ph_camera.imHeight())/2);

    StereoVisionApp::floatParameter pb_ppX(pFloatType(pb_camera.imWidth())/2);

    StereoVisionApp::floatParameter zero(0);

    ph_camera.setFLen(fLen);
    ph_camera.setOpticalCenterX(ph_ppX);
    ph_camera.setOpticalCenterY(ph_ppY);

    ph_camera.setB1(zero);
    ph_camera.setB2(zero);

    ph_camera.setP1(zero);
    ph_camera.setP2(zero);

    ph_camera.setK1(zero);
    ph_camera.setK2(zero);
    ph_camera.setK3(zero);
    ph_camera.setK4(zero);
    ph_camera.setK5(zero);
    ph_camera.setK6(zero);

    pb_camera.setFLen(fLen);
    pb_camera.setOpticalCenterX(pb_ppX);

    pb_camera.setA1(zero);
    pb_camera.setA2(zero);
    pb_camera.setA3(zero);
    pb_camera.setA4(zero);
    pb_camera.setA5(zero);

    pb_camera.setB1(zero);
    pb_camera.setB2(zero);
    pb_camera.setB3(zero);
    pb_camera.setB4(zero);
    pb_camera.setB5(zero);

    StereoVisionApp::PinholeCamProjModule ph_projectorModule(&ph_camera);
    StereoVisionApp::PinholePushBroomCamProjectorModule pb_projectorModule(&pb_camera);

    ph_projectorModule.setVerbose(false);
    ph_projectorModule.setup(&sbaSolver, problem);

    pb_projectorModule.setVerbose(false);
    pb_projectorModule.setup(&sbaSolver, problem);

    bool ph_initOk = ph_projectorModule.init();

    QVERIFY(ph_initOk);

    bool pb_initOk = pb_projectorModule.init();

    QVERIFY(pb_initOk);

    std::array<double, 3> pointPos = {42,0,fLen.value()};
    std::array<double, 3> ph_camRot = {0,0,0};
    std::array<double, 3> ph_camPos = {27,-33,0};

    std::array<double, 3> pb_camRot = {0,0,0};
    std::array<double, 3> pb_camPos = {0,0,0};

    Eigen::Vector2d ph_ptPos;
    ph_ptPos << pointPos[0] - ph_camPos[0] + ph_ppX.value(), pointPos[1] - ph_camPos[1] + ph_ppY.value();
    ph_ptPos /= pointPos[2];
    ph_ptPos *= fLen.value();

    Eigen::Vector2d pb_ptPos;
    pb_ptPos << pointPos[0] + pb_ppX.value(), pointPos[1];
    pb_ptPos /= pointPos[2];
    pb_ptPos *= fLen.value();

    Eigen::Matrix2d stiffness = Eigen::Matrix2d::Identity();

    using ProjectionInfos = StereoVisionApp::ModularSBASolver::ProjectorModule::ProjectionInfos;

    ProjectionInfos ph_projection_infos = ph_projectorModule.getProjectionInfos();
    ProjectionInfos pb_projection_infos = pb_projectorModule.getProjectionInfos();


    QVERIFY(ph_projection_infos.modularProjector != nullptr);
    QVERIFY(pb_projection_infos.modularProjector != nullptr);

    delete ph_projection_infos.modularProjector;
    delete pb_projection_infos.modularProjector;

    bool addingOk = StereoVisionApp::ModularSBASolver::ProjectorModule::addCrossProjectionCostFunction
        (
        &ph_projectorModule, ph_camRot.data(), ph_camPos.data(), ph_ptPos, stiffness,
        &pb_projectorModule, pb_camRot.data(), pb_camPos.data(), pb_ptPos, stiffness
        );


    QVERIFY(addingOk);

    //check the cost was effectively added
    QVERIFY(problem.NumResidualBlocks() >= 1);

    double cost;

    ceres::Problem::EvaluateOptions options;

    bool evaluateOk = problem.Evaluate(options, &cost, nullptr, nullptr, nullptr);

    QVERIFY(evaluateOk);

    QVERIFY(std::abs(cost) < 1e-6);

}


QTEST_MAIN(TestUvProjector);
#include "main.moc"
