#ifndef STEREOVISIONAPP_SBAINITIALIZER_H
#define STEREOVISIONAPP_SBAINITIALIZER_H

#include "datablocks/floatparameter.h"

#include <QMap>

#include <Eigen/Core>

namespace StereoVisionApp {

class Project;

class SBAInitializer
{
public:

	typedef Eigen::Vector3f Pt3D;

	struct Pt6D {
		Eigen::Vector3f t;
		Eigen::Matrix3f R;
	};

	struct InitialSolution {
		QMap<qint64, Pt3D> points;
		QMap<qint64, Pt6D> cams;
	};

	virtual ~SBAInitializer();

	virtual InitialSolution computeInitialSolution(Project* p, QSet<qint64> const& s_pts, QSet<qint64> const& s_imgs) = 0;

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
