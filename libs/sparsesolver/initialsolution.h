#ifndef STEREOVISIONAPP_INITIALSOLUTION_H
#define STEREOVISIONAPP_INITIALSOLUTION_H

#include <StereoVision/geometry/pointcloudalignment.h>

#include <QtCore>
#include <map>

namespace StereoVisionApp {

struct InitialSolution
{
public:

	typedef Eigen::Vector3f Pt3D;
	typedef std::map<qint64, Pt3D, std::less<qint64>, Eigen::aligned_allocator<std::pair<const qint64, Pt3D> > > PointMap;

	typedef StereoVision::Geometry::AffineTransform<float> Pt6D;

	typedef std::map<qint64, Pt6D, std::less<qint64>, Eigen::aligned_allocator<std::pair<const qint64, Pt6D> > > CamMap;

	InitialSolution();

	PointMap points;
	CamMap cams;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_INITIALSOLUTION_H
