#include <QtTest/QtTest>

#include <StereoVision/geometry/alignement.h>
#include <StereoVision/geometry/rotations.h>
#include <StereoVision/geometry/geometricexception.h>

#include <random>
#include <iostream>
#include <eigen3/Eigen/SVD>
#include <eigen3/Eigen/Geometry>

using namespace StereoVision;
using namespace StereoVision::Geometry;

Eigen::Array3Xf generateRandomPoints(int nPoints, float distance = 3.0, float spread = 2.0, float v_spread = 1.0) {

	Eigen::Array3Xf r;
	r.setRandom(3,nPoints);

	r.topRows(2) *= spread;
	r.row(2) *= v_spread;

	r.row(2) += distance;

	return r;
}

AffineTransform<float> generateRandomTransform(float distance = 3.0, float dist_variability = 0.5, float rot_perturbation = 0.1) {

	Eigen::Vector3f randLogRot;
	randLogRot.setRandom();

	if (randLogRot.isZero()) {
		randLogRot.setOnes();
	}

    Eigen::Matrix3f R = rodriguezFormula<float>(randLogRot*M_PI);
    Eigen::Matrix3f R_perturb = rodriguezFormula<float>(randLogRot*rot_perturbation);

	Eigen::Vector3f t;
	t.setZero();
	t[2] = distance;

	std::random_device rd;

	std::default_random_engine re(rd());
	std::uniform_real_distribution<float> uDist(-1., 1.);

	float dPerturb = uDist(re)*dist_variability;

	if (dPerturb < 1e-3) {
		dPerturb = dist_variability;
	}

	t = t - (1+dPerturb)*R*t;

    return AffineTransform<float>(R_perturb*R.transpose(), -R.transpose()*t);

}

class TestReprojectionMethods: public QObject
{
	Q_OBJECT
private Q_SLOTS:
	void testBuildEssentialMatrix_data();
	void testBuildEssentialMatrix();

	void testReprojection_data();
	void testReprojection();

	void testExtractTransform_data();
	void testExtractTransform();

	void testPnP_data();
	void testPnP();
};

void TestReprojectionMethods::testBuildEssentialMatrix_data() {

	QTest::addColumn<int>("nPts");
	QTest::addColumn<float>("dist");
	QTest::addColumn<float>("spread");
	QTest::addColumn<float>("v_spread");

	QTest::newRow("Not enough points") << 5 << 3.0f << 2.0f << 1.0f;
	QTest::newRow("Just enough points") << 8 << 3.0f << 2.0f << 1.0f;
	QTest::newRow("More than enough points") << 27 << 3.0f << 2.0f << 1.0f;
}

void TestReprojectionMethods::testBuildEssentialMatrix() {

	QFETCH(int, nPts);
	QFETCH(float, dist);
	QFETCH(float, spread);
	QFETCH(float, v_spread);

	Eigen::Array3Xf points = generateRandomPoints(nPts, dist, spread, v_spread);
	AffineTransform cam_delta = generateRandomTransform(dist);

	Eigen::Array2Xf pt_im1 = projectPoints(points);
    Eigen::Array2Xf pt_im2 = projectPoints<float>(points, cam_delta.R.transpose(), -cam_delta.R.transpose()*cam_delta.t);

	Eigen::Matrix3f E;

	if (nPts < 8) {
		QVERIFY_EXCEPTION_THROWN(E = estimateEssentialMatrix(pt_im1, pt_im2), GeometricException);
	} else {
		E = estimateEssentialMatrix(pt_im1, pt_im2);

		auto svd = E.jacobiSvd();

		float singularTopRatio = svd.singularValues()[0]/svd.singularValues()[1];
		float errorMargin = std::fabs(singularTopRatio - 1.0);

		QVERIFY2(errorMargin < 1e-3, qPrintable(QString("First singular value ratio too big (%1)").arg(singularTopRatio)));

		float lastSingularValue = std::fabs(svd.singularValues()[2]);
		QVERIFY2(lastSingularValue < 1e-3, qPrintable(QString("Last singular value too big (%1)").arg(lastSingularValue)));
	}
}

void TestReprojectionMethods::testReprojection_data() {

	QTest::addColumn<int>("nPts");
	QTest::addColumn<float>("dist");
	QTest::addColumn<float>("spread");
	QTest::addColumn<float>("v_spread");

	QTest::newRow("Near points") << 14 << 1.0f << 2.0f << 0.3f;
	QTest::newRow("Middle points") << 14 << 3.0f << 2.0f << 1.0f;
	QTest::newRow("Far points") << 14 << 7.0f << 2.0f << 1.0f;
	QTest::newRow("Flat points") << 14 << 3.0f << 2.0f << 0.05f;

}
void TestReprojectionMethods::testReprojection() {

	QFETCH(int, nPts);
	QFETCH(float, dist);
	QFETCH(float, spread);
	QFETCH(float, v_spread);

	Eigen::Array3Xf points = generateRandomPoints(nPts, dist, spread, v_spread);
	AffineTransform cam_delta = generateRandomTransform(dist);  //cam2 2 cam1

	Eigen::Array2Xf pt_im1 = projectPoints(points);
    Eigen::Array2Xf pt_im2 = projectPoints<float>(points, cam_delta.R.transpose(), -cam_delta.R.transpose()*cam_delta.t);

    Eigen::Array3Xf reprojected_points = reprojectPoints<float>(cam_delta.R.transpose(), -cam_delta.R.transpose()*cam_delta.t, pt_im1, pt_im2);

	float mismatch = (points - reprojected_points).matrix().norm()/nPts;
	QVERIFY2(mismatch < 1e-4, qPrintable(QString("Reprojected points not correct (%1)").arg(mismatch)));
}

void TestReprojectionMethods::testExtractTransform_data() {

	QTest::addColumn<int>("nPts");
	QTest::addColumn<float>("dist");
	QTest::addColumn<float>("spread");
	QTest::addColumn<float>("v_spread");

	QTest::newRow("Near points") << 14 << 1.7f << 1.5f << 1.0f;
	QTest::newRow("Middle points") << 14 << 3.0f << 2.0f << 1.0f;
	QTest::newRow("Far points") << 14 << 7.0f << 2.0f << 1.0f;
	QTest::newRow("Flat points") << 14 << 3.0f << 2.0f << 0.05f;

}
void TestReprojectionMethods::testExtractTransform() {

	QFETCH(int, nPts);
	QFETCH(float, dist);
	QFETCH(float, spread);
	QFETCH(float, v_spread);

	if (nPts < 8) {
		QSKIP("Not enough points to proceed");
	}

	Eigen::Array3Xf points = generateRandomPoints(nPts, dist, spread, v_spread);
	AffineTransform cam_delta = generateRandomTransform(dist); //cam2 2 cam1

	Eigen::Array3Xf pointsCam2 = cam_delta*points;

	if ((pointsCam2.row(2) < 0).any()) {
		QSKIP("Misconstructed random setup"); //TODO check the condition is correctly written...
	} else {

		Eigen::Array2Xf pt_im1 = projectPoints(points);
        Eigen::Array2Xf pt_im2 = projectPoints<float>(points, cam_delta.R.transpose(), -cam_delta.R.transpose()*cam_delta.t);

		Eigen::Matrix3f E = estimateEssentialMatrix(pt_im1, pt_im2);

        std::pair<AffineTransform<float>, AffineTransform<float>> candidates = essentialMatrix2Transforms(E);
		AffineTransform extractedTransform = selectTransform(candidates.first, candidates.second, pt_im1, pt_im2); //cam1 2 cam2

		Eigen::Matrix3f Rdelta = cam_delta.R*extractedTransform.R;
		Eigen::Vector3f tdelta = cam_delta.t/cam_delta.t.norm() + extractedTransform.R.transpose()*extractedTransform.t;

		float missalignement = (Rdelta - Eigen::Matrix3f::Identity()).norm();
		QVERIFY2(missalignement < 1e-3, qPrintable(QString("Reconstructed rotation not correct (norm (RgtxRrc - I) = %1)").arg(missalignement)));

		missalignement = tdelta.norm();
		QVERIFY2(missalignement < 1e-3, qPrintable(QString("Reconstructed rotation not correct (norm (tgt + Rrcxtrc) = %1)").arg(missalignement)));
	}
}

void TestReprojectionMethods::testPnP_data() {

	QTest::addColumn<int>("nPts");
	QTest::addColumn<float>("dist");
	QTest::addColumn<float>("spread");
	QTest::addColumn<float>("v_spread");

	QTest::newRow("Few points") << 4 << 3.5f << 2.0f << 2.0f;
	QTest::newRow("Some points") << 8 << 3.5f << 2.0f << 2.0f;
	QTest::newRow("Many points") << 12 << 3.5f << 2.0f << 2.0f;

}
void TestReprojectionMethods::testPnP() {

	QFETCH(int, nPts);
	QFETCH(float, dist);
	QFETCH(float, spread);
	QFETCH(float, v_spread);

	if (nPts < 4) {
		QSKIP("Not enough points to proceed");
	}

	Eigen::Array3Xf points = generateRandomPoints(nPts, dist, spread, v_spread);
	AffineTransform cam_delta = generateRandomTransform(dist); //cam 2 world
    AffineTransform inv = AffineTransform<float>(cam_delta.R.transpose(), -cam_delta.R.transpose()*cam_delta.t); //world 2 cam

	Eigen::Array3Xf pointsCam2 = inv*points;
	//std::cout << pointsCam2 << std::endl << std::endl;

	if ((pointsCam2.row(2) < 0).any()) {
		QSKIP("Misconstructed random setup");
	} else {
		Eigen::Array2Xf pt_im = projectPoints(points, inv);

		AffineTransform extractedTransform = pnp(pt_im, points); //world 2 cam

		/*pointsCam2 = extractedTransform*points;
		std::cout << pointsCam2 << std::endl << std::endl;*/

		Eigen::Matrix3f Rdelta = cam_delta.R*extractedTransform.R;
		Eigen::Vector3f tdelta = cam_delta.t + extractedTransform.R.transpose()*extractedTransform.t;

		float missalignement = (Rdelta - Eigen::Matrix3f::Identity()).norm();
		QVERIFY2(missalignement < 1e-3, qPrintable(QString("Reconstructed rotation not correct (norm (RgtxRrc - I) = %1)").arg(missalignement)));

		missalignement = tdelta.norm();
		QVERIFY2(missalignement < 1e-3, qPrintable(QString("Reconstructed rotation not correct (norm (tgt + Rrcxtrc) = %1)").arg(missalignement)));
	}
}

QTEST_MAIN(TestReprojectionMethods)
#include "testProjection.moc"
