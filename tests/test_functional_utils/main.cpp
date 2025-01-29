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

};

void TestFunctionalUtils::initTestCase() {

    srand(time(nullptr));

}

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

        std::array<double,6> r_in;
        std::array<double,6> t_in;

        std::array<double,6> r_in2;
        std::array<double,6> t_in2;

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

        auto verifyPoseIsZero = [this] (StereoVision::Geometry::RigidBodyTransform<double> const& pose) {


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


    }

}

QTEST_MAIN(TestFunctionalUtils);
#include "main.moc"
