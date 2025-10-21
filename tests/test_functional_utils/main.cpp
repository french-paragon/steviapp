#include <QtTest/QtTest>

#include "utils/functional.h"
#include "sparsesolver/costfunctors/posedecoratorfunctors.h"

#include <random>
#include <time.h>

#include <StereoVision/geometry/rotations.h>

class TestFunctionalUtils : public QObject
{
    Q_OBJECT
public:

private Q_SLOTS:

    void initTestCase();

    void testTupleCaller();

    void testPoseDecorators();

    void testPoseBuilderHelpers();

};

void TestFunctionalUtils::initTestCase() {

    srand(time(nullptr));

}

template<int nArgs, int copyArgId, int argSize>
struct DynamicIdentityArgFunctor {
    DynamicIdentityArgFunctor() {

    }

    template <typename T, typename ... P>
    bool operator()(T const* const* parameters, T* residuals) const {

        for (int i = 0; i < argSize; i++) {
            residuals[i] = parameters[copyArgId][i];
        }

        return true;
    }

};

void TestFunctionalUtils::testTupleCaller() {

    //test tuple manip
    std::tuple<int,double,int> strangeData(42,3.14159265,42);
    std::tuple<int,int> removed = StereoVisionApp::CallFromTuple::removeArgFromTuple<1>(strangeData);

    int charmed = 69;

    std::tuple<int,int&,int> charmedData = StereoVisionApp::CallFromTuple::insertArgInTuple<1,int&>(removed,charmed);

    QCOMPARE(std::get<1>(charmedData),69);

    charmed = 27;
    QCOMPARE(std::get<1>(charmedData),charmed);
    QCOMPARE(std::get<1>(charmedData),27);
    QCOMPARE(&std::get<1>(charmedData),&charmed);

    //test call to function
    std::get<0>(charmedData) = 33;
    std::get<2>(charmedData) = 6;

    auto callable = [] (int const& source, int& target, int retVal) {

        target = source;
        return retVal;
    };

    int val = StereoVisionApp::CallFromTuple::call(callable, charmedData);
    QCOMPARE(std::get<1>(charmedData),std::get<0>(charmedData));
    QCOMPARE(std::get<1>(charmedData),33);
    QCOMPARE(val,std::get<2>(charmedData));
    QCOMPARE(val,6);

}


void TestFunctionalUtils::testPoseDecorators() {

    class IdentityPose {
    public:
        IdentityPose() {

        }

        bool operator()(const double* r,
                        const double* t,
                        double* res) const {
            for (int i = 0; i < 3; i++) {
                res[i] = r[i];
                res[i+3] = t[i];
            }

            return true;
        }
    };

    constexpr int nRedos = 10;

    for (int i = 0; i < nRedos; i++) {

        StereoVision::Geometry::RigidBodyTransform<double> pose(Eigen::Vector3d::Random(), Eigen::Vector3d::Random());
        StereoVision::Geometry::RigidBodyTransform<double> pose2(Eigen::Vector3d::Random(), Eigen::Vector3d::Random());

        StereoVision::Geometry::RigidBodyTransform<double> outPose;

        std::array<double,3> r_in;
        std::array<double,3> t_in;

        std::array<double,3> r_in2;
        std::array<double,3> t_in2;

        for (int i = 0; i < 3; i++) {
            r_in[i] = pose.r[i];
            t_in[i] = pose.t[i];

            r_in2[i] = pose2.r[i];
            t_in2[i] = pose2.t[i];
        }

        std::array<double,6> out;

        auto reconfigureOutPose = [&out, &outPose] () {

            for (int i = 0; i < 3; i++) {
                outPose.r[i] = out[i];
                outPose.t[i] = out[i+3];
            }
        };

        auto verifyPoseIsZero = [] (StereoVision::Geometry::RigidBodyTransform<double> const& pose) {

            for (int i = 0; i < 3; i++) {
                QVERIFY(std::abs(pose.r[i]) < 1e-4);
                QVERIFY(std::abs(pose.t[i]) < 1e-4);
            }
        };

        IdentityPose indentity_functor;

        //Test invert pose decorator
        using Invert = StereoVisionApp::InvertPose<IdentityPose, 0>;

        Invert invert_functor;

        constexpr bool typeCheckInvert =
                std::is_same_v<decltype (invert_functor(r_in.data(), t_in.data(), out.data())),
                decltype (indentity_functor(r_in.data(), t_in.data(), out.data()))>;

        static_assert (typeCheckInvert, "Type of invert decorated function is not consistent with initial function");

        bool ok = invert_functor(r_in.data(), t_in.data(), out.data());

        QVERIFY2(ok, "Decorated invert function unexpectly returned false");

        reconfigureOutPose();

        StereoVision::Geometry::RigidBodyTransform<double> inverseDelta =
                pose*outPose;

        verifyPoseIsZero(inverseDelta);

        //test interpolate pose
        std::random_device rd;
        std::default_random_engine re;
        re.seed(rd());

        double w = std::uniform_real_distribution<double>(0,1)(re);

        StereoVision::Geometry::RigidBodyTransform<double> interpolated =
                StereoVision::Geometry::interpolateRigidBodyTransformOnManifold(w, pose,
                                                                                1 - w, pose2);

        using Interpolate = StereoVisionApp::InterpolatedPose<IdentityPose,0>;

        Interpolate interpolate_functor(w, 1-w);

        constexpr bool typeCheckInterpolate =
                std::is_same_v<decltype (interpolate_functor(r_in.data(), t_in.data(), r_in2.data(), t_in2.data(), out.data())),
                decltype (indentity_functor(r_in.data(), t_in.data(), out.data()))>;

        static_assert (typeCheckInterpolate, "Type of interpolate decorated function is not consistent with initial function");

        ok = interpolate_functor(r_in.data(), t_in.data(), r_in2.data(), t_in2.data(), out.data());

        QVERIFY2(ok, "Decorated interpolated function unexpectly returned false");

        reconfigureOutPose();

        inverseDelta =
                interpolated.inverse()*outPose;

        verifyPoseIsZero(inverseDelta);


        //test transform pose
        using TransformBefore = StereoVisionApp::PoseTransform<IdentityPose, StereoVisionApp::PoseTransformDirection::SourceToInitial, 0>;
        using TransformAfter = StereoVisionApp::PoseTransform<IdentityPose, StereoVisionApp::PoseTransformDirection::FinalToTarget, 0>;

        TransformBefore transform_before_functor(pose2);
        TransformAfter transform_after_functor(pose2);

        constexpr bool typeCheckTransform =
                std::is_same_v<decltype (transform_before_functor(r_in.data(), t_in.data(), out.data())),
                decltype (indentity_functor(r_in.data(), t_in.data(), out.data()))> and
                std::is_same_v<decltype (transform_after_functor(r_in.data(), t_in.data(), out.data())),
                decltype (indentity_functor(r_in.data(), t_in.data(), out.data()))>;

        static_assert (typeCheckTransform, "Type of transform decorated functions is not consistent with initial function");

        ok = transform_before_functor(r_in.data(), t_in.data(), out.data());

        QVERIFY2(ok, "Decorated transform before function unexpectly returned false");

        reconfigureOutPose();

        inverseDelta = pose2.inverse()*pose.inverse()*outPose;
        verifyPoseIsZero(inverseDelta);

        ok = transform_after_functor(r_in.data(), t_in.data(), out.data());

        QVERIFY2(ok, "Decorated transform after function unexpectly returned false");

        reconfigureOutPose();

        inverseDelta = pose.inverse()*pose2.inverse()*outPose;
        verifyPoseIsZero(inverseDelta);

        //test lever arm pose

        constexpr int B2WS2B = StereoVisionApp::Sensor2Body | StereoVisionApp::Body2World;
        constexpr int W2BS2B = StereoVisionApp::Sensor2Body | StereoVisionApp::World2Body;
        constexpr int B2WB2S = StereoVisionApp::Body2Sensor | StereoVisionApp::Body2World;
        constexpr int W2BB2S = StereoVisionApp::Body2Sensor | StereoVisionApp::World2Body;

        using LeverArmB2WS2B = StereoVisionApp::LeverArm<IdentityPose,B2WS2B,0>;
        using LeverArmW2BS2B = StereoVisionApp::LeverArm<IdentityPose,W2BS2B,0>;
        using LeverArmB2WB2S = StereoVisionApp::LeverArm<IdentityPose,B2WB2S,0>;
        using LeverArmW2BB2S = StereoVisionApp::LeverArm<IdentityPose,W2BB2S,0>;

        LeverArmB2WS2B leverarm_B2WS2B_functor;
        LeverArmW2BS2B leverarm_W2BS2B_functor;
        LeverArmB2WB2S leverarm_B2WB2S_functor;
        LeverArmW2BB2S leverarm_W2BB2S_functor;

        constexpr bool typeCheckLeverArm =
                std::is_same_v<decltype (leverarm_B2WS2B_functor(r_in.data(), t_in.data(), r_in2.data(), t_in2.data(), out.data())),
                decltype (indentity_functor(r_in.data(), t_in.data(), out.data()))> and
                std::is_same_v<decltype (leverarm_W2BS2B_functor(r_in.data(), t_in.data(), r_in2.data(), t_in2.data(), out.data())),
                decltype (indentity_functor(r_in.data(), t_in.data(), out.data()))> and
                std::is_same_v<decltype (leverarm_B2WB2S_functor(r_in.data(), t_in.data(), r_in2.data(), t_in2.data(), out.data())),
                decltype (indentity_functor(r_in.data(), t_in.data(), out.data()))> and
                std::is_same_v<decltype (leverarm_W2BB2S_functor(r_in.data(), t_in.data(), r_in2.data(), t_in2.data(), out.data())),
                decltype (indentity_functor(r_in.data(), t_in.data(), out.data()))>;

        static_assert (typeCheckLeverArm, "Type of lever arm decorated functions is not consistent with initial function");

        ok = leverarm_B2WS2B_functor(r_in.data(), t_in.data(), r_in2.data(), t_in2.data(), out.data());

        QVERIFY2(ok, "Decorated lever arm B2WS2B function unexpectly returned false");

        reconfigureOutPose();

        inverseDelta = (pose*pose2) * outPose.inverse();
        verifyPoseIsZero(inverseDelta);


        ok = leverarm_W2BS2B_functor(r_in.data(), t_in.data(), r_in2.data(), t_in2.data(), out.data());

        QVERIFY2(ok, "Decorated lever arm W2BS2B function unexpectly returned false");

        reconfigureOutPose();

        inverseDelta = (pose2.inverse()*pose) * outPose.inverse();
        verifyPoseIsZero(inverseDelta);


        ok = leverarm_B2WB2S_functor(r_in.data(), t_in.data(), r_in2.data(), t_in2.data(), out.data());

        QVERIFY2(ok, "Decorated lever arm B2WB2S function unexpectly returned false");

        reconfigureOutPose();

        inverseDelta = (pose*pose2.inverse()) * outPose.inverse();
        verifyPoseIsZero(inverseDelta);


        ok = leverarm_W2BB2S_functor(r_in.data(), t_in.data(), r_in2.data(), t_in2.data(), out.data());

        QVERIFY2(ok, "Decorated lever arm W2BB2S function unexpectly returned false");

        reconfigureOutPose();

        inverseDelta = (pose2*pose) * outPose.inverse();
        verifyPoseIsZero(inverseDelta);

        using WithMoreOrientation = StereoVisionApp::AddOrientation<IdentityPose,1>;

        WithMoreOrientation with_orientation_functor;

        constexpr bool typeCheckWithMoreOrientation =
            std::is_same_v<decltype (with_orientation_functor(r_in.data(), r_in2.data(), t_in.data(), out.data())),
                           decltype (indentity_functor(r_in.data(), t_in.data(), out.data()))>;

        static_assert (typeCheckWithMoreOrientation, "Type of with more orientation decorated function is not consistent with initial function");

        ok = with_orientation_functor(r_in.data(), r_in2.data(), t_in.data(), out.data());

        QVERIFY2(ok, "Decorated with more orientation function unexpectly returned false");

        reconfigureOutPose();

        StereoVision::Geometry::RigidBodyTransform<double> withMoreOrientationDelta =
            pose*outPose.inverse();

        verifyPoseIsZero(withMoreOrientationDelta);


        using WithMorePose = StereoVisionApp::AddPose<IdentityPose,0>;

        WithMorePose with_more_pose_functor;

        constexpr bool typeCheckWithMorePose =
            std::is_same_v<decltype (with_more_pose_functor(r_in.data(), t_in.data(), r_in2.data(), t_in2.data(), out.data())),
                           decltype (indentity_functor(r_in.data(), t_in.data(), out.data()))>;

        static_assert (typeCheckWithMorePose,
                      "Type of with lever arm and more orientation decorated function is not consistent with initial function");

        ok = with_more_pose_functor(r_in.data(), t_in.data(), r_in2.data(), t_in2.data(),out.data());

        QVERIFY2(ok, "Decorated with more orientation function unexpectly returned false");

        reconfigureOutPose();

        StereoVision::Geometry::RigidBodyTransform<double> withMorePoseDelta =
            pose2*outPose.inverse();

        verifyPoseIsZero(withMorePoseDelta);

        constexpr int poseConfig = StereoVisionApp::Sensor2Body | StereoVisionApp::Body2World;
        using WithMounting = StereoVisionApp::ApplyLeverArm<WithMorePose, 0, 2, poseConfig>;

        WithMounting with_mounting;

        constexpr bool typeCheckWithMounting =
            std::is_same_v<decltype (with_mounting(r_in.data(), t_in.data(), r_in2.data(), t_in2.data(), out.data())),
                           decltype (indentity_functor(r_in.data(), t_in.data(), out.data()))>;

        static_assert (typeCheckWithMounting,
                      "Type of with lever arm and more orientation decorated function is not consistent with initial function");

        ok = with_mounting(r_in.data(), t_in.data(), r_in2.data(), t_in2.data(),out.data());

        QVERIFY2(ok, "Decorated with more orientation function unexpectly returned false");

        reconfigureOutPose();

        StereoVision::Geometry::RigidBodyTransform<double> p2p = pose2*pose;
        StereoVision::Geometry::RigidBodyTransform<double> pp2 = pose*pose2;

        StereoVision::Geometry::RigidBodyTransform<double> withMountingDelta =
            (pose2*pose)*outPose.inverse();

        verifyPoseIsZero(withMountingDelta);


    }

}



void TestFunctionalUtils::testPoseBuilderHelpers() {

    using SinglePoseRotCopyFunc = DynamicIdentityArgFunctor<2, 0, 3>;
    using SinglePosePosCopyFunc = DynamicIdentityArgFunctor<2, 1, 3>;

    constexpr int nRuns = 5;

    for (int i = 0; i < nRuns; i++) {

        StereoVision::Geometry::RigidBodyTransform<double> identity(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

        StereoVision::Geometry::RigidBodyTransform<double> pose(Eigen::Vector3d::Random(), Eigen::Vector3d::Random());
        StereoVision::Geometry::RigidBodyTransform<double> pose2(Eigen::Vector3d::Random(), Eigen::Vector3d::Random());
        StereoVision::Geometry::RigidBodyTransform<double> pose3(Eigen::Vector3d::Random(), Eigen::Vector3d::Random());

        StereoVision::Geometry::RigidBodyTransform<double> outPose;
        StereoVision::Geometry::RigidBodyTransform<double> expectedOutPose;

        std::array<double,3> r_in;
        std::array<double,3> t_in;

        std::array<double,3> r_in2;
        std::array<double,3> t_in2;

        for (int i = 0; i < 3; i++) {
            r_in[i] = pose.r[i];
            t_in[i] = pose.t[i];

            r_in2[i] = pose2.r[i];
            t_in2[i] = pose2.t[i];
        }

        std::array<double,3> r_out;
        std::array<double,3> t_out;

        std::vector<double*> parameters = {r_in.data(), t_in.data(), r_in2.data(), t_in2.data()};
        std::vector<int> baseParametersSizeInfos = {3,3};
        std::vector<int> joinParametersSizeInfos = {3,3,3,3};

        constexpr StereoVisionApp::PoseTransformDirection poseTransformDirection =
            StereoVisionApp::PoseTransformDirection::SourceToInitial;

        constexpr int LeverArmConfiguration = StereoVisionApp::Body2World|StereoVisionApp::Sensor2Body;

        constexpr int poseParamIdx = 0;
        constexpr int nResiduals = 3;

        constexpr int stride = 4;

        using SingleRotBuildHelper = StereoVisionApp::ModifiedPoseCostFunctionBuilderHelper
            <SinglePoseRotCopyFunc, poseTransformDirection, LeverArmConfiguration, poseParamIdx, nResiduals>;

        using SinglePosBuildHelper = StereoVisionApp::ModifiedPoseCostFunctionBuilderHelper
            <SinglePosePosCopyFunc, poseTransformDirection, LeverArmConfiguration, poseParamIdx, nResiduals>;

        //with nothing

        auto singleRotFunctionData = SingleRotBuildHelper::buildPoseShiftedDynamicCostFunction<stride>
            (
            parameters.data(),
            baseParametersSizeInfos,
            identity,
            nullptr,
            nullptr);

        auto singlePosFunctionData = SinglePosBuildHelper::buildPoseShiftedDynamicCostFunction<stride>
            (
                parameters.data(),
                baseParametersSizeInfos,
                identity,
                nullptr,
                nullptr);

        QVERIFY(singleRotFunctionData.costFunction != nullptr);
        QVERIFY(singlePosFunctionData.costFunction != nullptr);

        QCOMPARE(singleRotFunctionData.params.size(), baseParametersSizeInfos.size());

        QCOMPARE(singlePosFunctionData.params.size(), baseParametersSizeInfos.size());

        QCOMPARE(singleRotFunctionData.costFunction->parameter_block_sizes().size(), singleRotFunctionData.params.size());
        QCOMPARE(singlePosFunctionData.costFunction->parameter_block_sizes().size(), singlePosFunctionData.params.size());

        bool rotOk = singleRotFunctionData.costFunction->Evaluate(singleRotFunctionData.params.data(), r_out.data(), nullptr);
        bool posOk = singlePosFunctionData.costFunction->Evaluate(singlePosFunctionData.params.data(), t_out.data(), nullptr);

        QVERIFY(rotOk);
        QVERIFY(posOk);

        for (int i = 0; i < 3; i++) {
            QCOMPARE(r_out[i], r_in[i]);
            QCOMPARE(t_out[i], t_in[i]);
        }

        delete singleRotFunctionData.costFunction;
        delete singlePosFunctionData.costFunction;

        //with a rigid transform

        singleRotFunctionData = SingleRotBuildHelper::buildPoseShiftedDynamicCostFunction<stride>
            (
                parameters.data(),
                baseParametersSizeInfos,
                pose2,
                nullptr,
                nullptr);

        singlePosFunctionData = SinglePosBuildHelper::buildPoseShiftedDynamicCostFunction<stride>
            (
                parameters.data(),
                baseParametersSizeInfos,
                pose2,
                nullptr,
                nullptr);

        QVERIFY(singleRotFunctionData.costFunction != nullptr);
        QVERIFY(singlePosFunctionData.costFunction != nullptr);

        QCOMPARE(singleRotFunctionData.params.size(), baseParametersSizeInfos.size());
        QCOMPARE(singlePosFunctionData.params.size(), baseParametersSizeInfos.size());

        QCOMPARE(singleRotFunctionData.costFunction->parameter_block_sizes().size(), singleRotFunctionData.params.size());
        QCOMPARE(singlePosFunctionData.costFunction->parameter_block_sizes().size(), singlePosFunctionData.params.size());

        rotOk = singleRotFunctionData.costFunction->Evaluate(singleRotFunctionData.params.data(), r_out.data(), nullptr);
        posOk = singlePosFunctionData.costFunction->Evaluate(singlePosFunctionData.params.data(), t_out.data(), nullptr);

        QVERIFY(rotOk);
        QVERIFY(posOk);

        expectedOutPose = pose*pose2;

        for (int i = 0; i < 3; i++) {
            QCOMPARE(r_out[i], expectedOutPose.r[i]);
            QCOMPARE(t_out[i], expectedOutPose.t[i]);
        }

        delete singleRotFunctionData.costFunction;
        delete singlePosFunctionData.costFunction;

        //with lever arm

        singleRotFunctionData = SingleRotBuildHelper::buildPoseShiftedDynamicCostFunction<stride>
            (
                parameters.data(),
                baseParametersSizeInfos,
                identity,
                r_in2.data(),
                t_in2.data());

        singlePosFunctionData = SinglePosBuildHelper::buildPoseShiftedDynamicCostFunction<stride>
            (
                parameters.data(),
                baseParametersSizeInfos,
                identity,
                r_in2.data(),
                t_in2.data());

        QVERIFY(singleRotFunctionData.costFunction != nullptr);
        QVERIFY(singlePosFunctionData.costFunction != nullptr);

        QCOMPARE(singleRotFunctionData.params.size(), joinParametersSizeInfos.size());
        QCOMPARE(singlePosFunctionData.params.size(), joinParametersSizeInfos.size());

        QCOMPARE(singleRotFunctionData.costFunction->parameter_block_sizes().size(), singleRotFunctionData.params.size());
        QCOMPARE(singlePosFunctionData.costFunction->parameter_block_sizes().size(), singlePosFunctionData.params.size());

        rotOk = singleRotFunctionData.costFunction->Evaluate(singleRotFunctionData.params.data(), r_out.data(), nullptr);
        posOk = singlePosFunctionData.costFunction->Evaluate(singlePosFunctionData.params.data(), t_out.data(), nullptr);

        QVERIFY(rotOk);
        QVERIFY(posOk);

        expectedOutPose = pose*pose2;

        for (int i = 0; i < 3; i++) {
            QCOMPARE(r_out[i], expectedOutPose.r[i]);
            QCOMPARE(t_out[i], expectedOutPose.t[i]);
        }

    }

    using MultiPoseRot1CopyFunc = DynamicIdentityArgFunctor<4, 0, 3>;
    using MultiPosePos1CopyFunc = DynamicIdentityArgFunctor<4, 1, 3>;

    using MultiPoseRot2CopyFunc = DynamicIdentityArgFunctor<4, 2, 3>;
    using MultiPosePos2CopyFunc = DynamicIdentityArgFunctor<4, 3, 3>;

    for (int i = 0; i < nRuns; i++) {

        StereoVision::Geometry::RigidBodyTransform<double> identity(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

        StereoVision::Geometry::RigidBodyTransform<double> pose1(Eigen::Vector3d::Random(), Eigen::Vector3d::Random());
        StereoVision::Geometry::RigidBodyTransform<double> pose2(Eigen::Vector3d::Random(), Eigen::Vector3d::Random());
        StereoVision::Geometry::RigidBodyTransform<double> pose3(Eigen::Vector3d::Random(), Eigen::Vector3d::Random());
        StereoVision::Geometry::RigidBodyTransform<double> pose4(Eigen::Vector3d::Random(), Eigen::Vector3d::Random());

        StereoVision::Geometry::RigidBodyTransform<double> outPose;
        StereoVision::Geometry::RigidBodyTransform<double> expectedOutPose;

        std::array<double,3> r_in1;
        std::array<double,3> t_in1;

        std::array<double,3> r_in2;
        std::array<double,3> t_in2;

        std::array<double,3> r_in3;
        std::array<double,3> t_in3;

        std::array<double,3> r_in4;
        std::array<double,3> t_in4;

        for (int i = 0; i < 3; i++) {
            r_in1[i] = pose1.r[i];
            t_in1[i] = pose1.t[i];

            r_in2[i] = pose2.r[i];
            t_in2[i] = pose2.t[i];

            r_in3[i] = pose3.r[i];
            t_in3[i] = pose3.t[i];

            r_in4[i] = pose4.r[i];
            t_in4[i] = pose4.t[i];
        }

        std::array<double,3> r_out1;
        std::array<double,3> t_out1;

        std::array<double,3> r_out2;
        std::array<double,3> t_out2;

        std::vector<double*> parameters = {r_in1.data(), t_in1.data(), r_in2.data(), t_in2.data()};
        std::vector<int> baseParametersSizeInfos = {3,3,3,3};
        std::vector<int> joinParametersSizeInfos = {3,3,3,3,3,3,3,3};

        constexpr StereoVisionApp::PoseTransformDirection poseTransformDirection =
            StereoVisionApp::PoseTransformDirection::SourceToInitial;

        constexpr int LeverArmConfiguration = StereoVisionApp::Body2World|StereoVisionApp::Sensor2Body;

        using poseParamIdxs = std::index_sequence<0,2>;
        constexpr int nResiduals = 3;

        constexpr int stride = 4;

        using MultiRot1BuildHelper = StereoVisionApp::ModifiedMultiPoseCostFunctionBuilderHelper
            <MultiPoseRot1CopyFunc, poseTransformDirection, LeverArmConfiguration, poseParamIdxs, nResiduals>;

        using MultiPos1BuildHelper = StereoVisionApp::ModifiedMultiPoseCostFunctionBuilderHelper
            <MultiPosePos1CopyFunc, poseTransformDirection, LeverArmConfiguration, poseParamIdxs, nResiduals>;

        using MultiRot2BuildHelper = StereoVisionApp::ModifiedMultiPoseCostFunctionBuilderHelper
            <MultiPoseRot2CopyFunc, poseTransformDirection, LeverArmConfiguration, poseParamIdxs, nResiduals>;

        using MultiPos2BuildHelper = StereoVisionApp::ModifiedMultiPoseCostFunctionBuilderHelper
            <MultiPosePos2CopyFunc, poseTransformDirection, LeverArmConfiguration, poseParamIdxs, nResiduals>;

        struct PoseInfosData {
            const StereoVision::Geometry::RigidBodyTransform<double> offset; //fixed offset applied on top of the parametrized pose.
            double *leverArmOrientation; //parameter for the lever arm orientation
            double *leverArmPosition; //parameter for the lever arm orientation
        };

        auto evaluateMulti = [&] (PoseInfosData const& pose1Infos,
                                 PoseInfosData const& pose2Infos,
                                 StereoVision::Geometry::RigidBodyTransform<double> const& expectedOut1,
                                 StereoVision::Geometry::RigidBodyTransform<double> const& expectedOut2) {

            auto multiRot1FunctionData = MultiRot1BuildHelper::buildPoseShiftedDynamicCostFunction<stride>
                (
                    parameters.data(),
                    baseParametersSizeInfos,
                    MultiRot1BuildHelper::PoseDataContainer{
                                                            MultiRot1BuildHelper::PoseModificationData{pose1Infos.offset, pose1Infos.leverArmOrientation, pose1Infos.leverArmPosition},
                                                            MultiRot1BuildHelper::PoseModificationData{pose2Infos.offset, pose2Infos.leverArmOrientation, pose2Infos.leverArmPosition}}
                    );

            auto multiPos1FunctionData = MultiPos1BuildHelper::buildPoseShiftedDynamicCostFunction<stride>
                (
                    parameters.data(),
                    baseParametersSizeInfos,
                    MultiPos1BuildHelper::PoseDataContainer{
                                                            MultiPos1BuildHelper::PoseModificationData{pose1Infos.offset, pose1Infos.leverArmOrientation, pose1Infos.leverArmPosition},
                                                            MultiPos1BuildHelper::PoseModificationData{pose2Infos.offset, pose2Infos.leverArmOrientation, pose2Infos.leverArmPosition}});

            auto multiRot2FunctionData = MultiRot2BuildHelper::buildPoseShiftedDynamicCostFunction<stride>
                (
                    parameters.data(),
                    baseParametersSizeInfos,
                    MultiRot2BuildHelper::PoseDataContainer{
                                                            MultiRot2BuildHelper::PoseModificationData{pose1Infos.offset, pose1Infos.leverArmOrientation, pose1Infos.leverArmPosition},
                                                            MultiRot2BuildHelper::PoseModificationData{pose2Infos.offset, pose2Infos.leverArmOrientation, pose2Infos.leverArmPosition}}
                    );

            auto multiPos2FunctionData = MultiPos2BuildHelper::buildPoseShiftedDynamicCostFunction<stride>
                (
                    parameters.data(),
                    baseParametersSizeInfos,
                    MultiPos2BuildHelper::PoseDataContainer{
                                                            MultiPos2BuildHelper::PoseModificationData{pose1Infos.offset, pose1Infos.leverArmOrientation, pose1Infos.leverArmPosition},
                                                            MultiPos2BuildHelper::PoseModificationData{pose2Infos.offset, pose2Infos.leverArmOrientation, pose2Infos.leverArmPosition}});

            QVERIFY(multiRot1FunctionData.costFunction != nullptr);
            QVERIFY(multiPos1FunctionData.costFunction != nullptr);
            QVERIFY(multiRot2FunctionData.costFunction != nullptr);
            QVERIFY(multiPos2FunctionData.costFunction != nullptr);

            int expectedSize = baseParametersSizeInfos.size();

            if (pose1Infos.leverArmOrientation != nullptr and pose1Infos.leverArmPosition != nullptr) {
                expectedSize += 2;
            }

            if (pose2Infos.leverArmOrientation != nullptr and pose2Infos.leverArmPosition != nullptr) {
                expectedSize += 2;
            }

            QCOMPARE(multiRot1FunctionData.params.size(), expectedSize);
            QCOMPARE(multiPos1FunctionData.params.size(), expectedSize);
            QCOMPARE(multiRot2FunctionData.params.size(), expectedSize);
            QCOMPARE(multiPos2FunctionData.params.size(), expectedSize);

            QCOMPARE(multiRot1FunctionData.costFunction->parameter_block_sizes().size(), multiRot1FunctionData.params.size());
            QCOMPARE(multiPos1FunctionData.costFunction->parameter_block_sizes().size(), multiPos1FunctionData.params.size());
            QCOMPARE(multiRot2FunctionData.costFunction->parameter_block_sizes().size(), multiRot1FunctionData.params.size());
            QCOMPARE(multiPos2FunctionData.costFunction->parameter_block_sizes().size(), multiPos1FunctionData.params.size());

            bool rot1Ok = multiRot1FunctionData.costFunction->Evaluate(multiRot1FunctionData.params.data(), r_out1.data(), nullptr);
            bool pos1Ok = multiPos1FunctionData.costFunction->Evaluate(multiPos1FunctionData.params.data(), t_out1.data(), nullptr);
            bool rot2Ok = multiRot2FunctionData.costFunction->Evaluate(multiRot2FunctionData.params.data(), r_out2.data(), nullptr);
            bool pos2Ok = multiPos2FunctionData.costFunction->Evaluate(multiPos2FunctionData.params.data(), t_out2.data(), nullptr);

            QVERIFY(rot1Ok);
            QVERIFY(pos1Ok);
            QVERIFY(rot2Ok);
            QVERIFY(pos2Ok);

            for (int i = 0; i < 3; i++) {
                QCOMPARE(r_out1[i], expectedOut1.r[i]);
                QCOMPARE(t_out1[i], expectedOut1.t[i]);
                QCOMPARE(r_out2[i], expectedOut2.r[i]);
                QCOMPARE(t_out2[i], expectedOut2.t[i]);
            }

            delete multiRot1FunctionData.costFunction;
            delete multiPos1FunctionData.costFunction;
            delete multiRot2FunctionData.costFunction;
            delete multiPos2FunctionData.costFunction;

        };

        //with nothing
        evaluateMulti({identity, nullptr, nullptr}, {identity, nullptr, nullptr}, pose1, pose2);

        //with fixed transform
        evaluateMulti({pose3, nullptr, nullptr}, {pose4, nullptr, nullptr}, pose1*pose3, pose2*pose4);

        //with lever arm transform
        evaluateMulti({identity, r_in3.data(), t_in3.data()}, {identity, r_in4.data(), t_in4.data()}, pose1*pose3, pose2*pose4);

        //with combined transform
        evaluateMulti({pose4, r_in3.data(), t_in3.data()}, {pose3, r_in4.data(), t_in4.data()}, pose1*pose3*pose4, pose2*pose4*pose3);

    }

}

QTEST_MAIN(TestFunctionalUtils);
#include "main.moc"
