#include "edgepointsangle.h"

#include <g2o/types/sba/types_sba.h>

#include <QDebug>

namespace StereoVisionApp {

EdgePointsAngle::EdgePointsAngle()
{
	resize(3);
}

bool EdgePointsAngle::read(std::istream& is) {
	Measurement q;
	is >> q;
	setMeasurement(q);
	for (int i = 0; i < Dimension; ++i)
		for (int j = i; j < Dimension; ++j) {
			is >> information()(i, j);
			if (i != j)
				information()(j, i) = information()(i, j);
		}
	return true;
}
bool EdgePointsAngle::write(std::ostream& os) const {
	Measurement q = measurement();
	os << q << " ";
	for (int i = 0; i < Dimension; ++i)
		for (int j = i; j < Dimension; ++j)
			os << " " << information()(i, j);
	return os.good();
}

void EdgePointsAngle::computeError() {

	const g2o::VertexPointXYZ* point1 = static_cast<const g2o::VertexPointXYZ*>(_vertices[0]);
	const g2o::VertexPointXYZ* point2 = static_cast<const g2o::VertexPointXYZ*>(_vertices[1]);
	const g2o::VertexPointXYZ* point3 = static_cast<const g2o::VertexPointXYZ*>(_vertices[2]);

	g2o::VertexPointXYZ::EstimateType pt1 = point1->estimate();
	g2o::VertexPointXYZ::EstimateType pt2 = point2->estimate();
	g2o::VertexPointXYZ::EstimateType pt3 = point3->estimate();

	Eigen::Vector3d pt2to1 = pt1 - pt2;
	Eigen::Vector3d pt2to3 = pt3 - pt2;

	double cosPoints = pt2to1.dot(pt2to3)/(pt2to1.norm()*pt2to3.norm());
	double sinPoints = std::sqrt(1 - cosPoints*cosPoints);

	double cosMeasure = std::cos(_measurement);
	double sinMeasure = std::sqrt(1 - cosMeasure*cosMeasure);

	double eCos = cosPoints - cosMeasure;
	double eSin = sinPoints - sinMeasure;

	if (!std::isfinite(eCos) or !std::isfinite(eSin)) { // points are at the same coordinates
		_error[0] = 0;
	} else {
		_error[0] = std::sqrt(eCos*eCos + eSin*eSin);
	}

	if (_error.hasNaN()) {
		qDebug() << "EdgePointsAngle";
	}

}

} // namespace StereoVisionApp
