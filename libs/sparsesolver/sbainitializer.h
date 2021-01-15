#ifndef STEREOVISIONAPP_SBAINITIALIZER_H
#define STEREOVISIONAPP_SBAINITIALIZER_H

#include "datablocks/floatparameter.h"

#include "geometry/pointcloudalignment.h"

#include <map>
#include <Eigen/Core>

namespace StereoVisionApp {

class Project;
class Image;

class SBAInitializer
{
public:

	typedef Eigen::Vector3f Pt3D;
	typedef std::map<qint64, Pt3D, std::less<qint64>, Eigen::aligned_allocator<std::pair<const qint64, Pt3D> > > PointMap;

	typedef StereoVision::Geometry::AffineTransform Pt6D;

	typedef std::map<qint64, Pt6D, std::less<qint64>, Eigen::aligned_allocator<std::pair<const qint64, Pt6D> > > CamMap;

	struct InitialSolution {
		PointMap points;
		CamMap cams;
	};

	virtual ~SBAInitializer();

	virtual InitialSolution computeInitialSolution(Project* p, QSet<qint64> const& s_pts, QSet<qint64> const& s_imgs) = 0;

};

class PhotometricInitializer : public SBAInitializer {

protected:

	static Eigen::Array3Xf getLandmarksWorldCoordinates(const InitialSolution &sol, QVector<qint64> const& idxs);

	static Eigen::Matrix3f ApproximateEssentialMatrix(Image* im1, Image* im2, QSet<qint64> const& intersection);
	static Eigen::Array2Xf getHomogeneousImageCoordinates(Image* im, QVector<qint64> ids);

	static StereoVision::Geometry::AffineTransform estimateTransform(InitialSolution const& solution, Project* p, bool useConstraintsRefinement = false);

	static bool completeSolution(InitialSolution & solution,
								 Project* p, QSet<qint64>
								 const& s_pts, QSet<qint64>
								 const& s_imgs,
								 int minNTiePoints = 4,
								 int minViewingImgs = 2);
	static bool alignImage(InitialSolution & solution, Project* p, Image *img, QSet<qint64> &pts);
	static bool triangulatePoint(InitialSolution & solution, Project* p, qint64 pt, QVector<qint64> const& imgs);

};

class EightPointsSBAMultiviewInitializer : public PhotometricInitializer
{
public:

	EightPointsSBAMultiviewInitializer(qint64 f1 = -1,
									   bool preconstrain = true,
									   bool useConstraintsRefinement = false);

	virtual InitialSolution computeInitialSolution(Project* p, QSet<qint64> const& s_pts, QSet<qint64> const& s_imgs);

private:

	qint64 _f1;

	bool _preconstrain;
	bool _useConstraintsRefinement;

};

class EightPointsSBAInitializer : public PhotometricInitializer
{
public:

	enum FrameSelectionBehavior : qint64 {
		AutoPointNumber = -1,
		AutoMatrixQuality = -2
	};

	EightPointsSBAInitializer(qint64 f1 = AutoMatrixQuality,
							  qint64 f2 = AutoMatrixQuality,
							  bool preconstrain = true,
							  bool useConstraintsRefinement = false);

	virtual InitialSolution computeInitialSolution(Project* p, QSet<qint64> const& s_pts, QSet<qint64> const& s_imgs);

private:
	static float scoreApproximateEssentialMatrix(Eigen::Matrix3f const& Eapprox);

	qint64 _f1;
	qint64 _f2;

	bool _preconstrain;
	bool _useConstraintsRefinement;
};

class FrontCamSBAInitializer : public SBAInitializer
{
public:

	static const int MinimalNObs;

	FrontCamSBAInitializer(int min_n_obs = 6, int rot_adjustement_steps = 4);

	virtual InitialSolution computeInitialSolution(Project* p, QSet<qint64> const& s_pts, QSet<qint64> const& s_imgs);

private:

	int _min_n_obs;
	int _rot_adjustement_steps;

};

class RandomPosSBAInitializer : public SBAInitializer
{
public:
	RandomPosSBAInitializer(float camDist = 2, float camStd = 0.1, float pointStd = 0.1);

	virtual InitialSolution computeInitialSolution(Project* p, QSet<qint64> const& s_pts, QSet<qint64> const& s_imgs);

private:

	float _camDist;
	float _camStd;
	float _pointStd;
};



} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_SBAINITIALIZER_H
