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

	typedef AffineTransform Pt6D;

	typedef std::map<qint64, Pt6D, std::less<qint64>, Eigen::aligned_allocator<std::pair<const qint64, Pt6D> > > CamMap;

	struct InitialSolution {
		PointMap points;
		CamMap cams;
	};

	virtual ~SBAInitializer();

	virtual InitialSolution computeInitialSolution(Project* p, QSet<qint64> const& s_pts, QSet<qint64> const& s_imgs) = 0;

};

class EightPointsSBAInitializer : public SBAInitializer
{
public:

	enum FrameSelectionBehavior : qint64 {
		AutoPointNumber = -1,
		AutoMatrixQuality = -2
	};

	EightPointsSBAInitializer(qint64 f1 = AutoMatrixQuality,
							  qint64 f2 = AutoMatrixQuality,
							  int triangulation_threshold = -1,
							  bool preconstrain = false);

	virtual InitialSolution computeInitialSolution(Project* p, QSet<qint64> const& s_pts, QSet<qint64> const& s_imgs);

private:

	static Eigen::Matrix3f ApproximateEssentialMatrix(Image* im1, Image* im2, QSet<qint64> const& intersection);
	static float scoreApproximateEssentialMatrix(Eigen::Matrix3f const& Eapprox);
	static Eigen::Array2Xf getHomogeneousImageCoordinates(Image* im, QVector<qint64> ids);

	static AffineTransform estimateTransform(InitialSolution const& solution, Project* p);

	qint64 _f1;
	qint64 _f2;
	int _auto_triangulation_threshold;

	bool _preconstrain;
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
