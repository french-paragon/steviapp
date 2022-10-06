#ifndef EDGEXYZPRIOR_H
#define EDGEXYZPRIOR_H

#include <eigen3/Eigen/Core>
#include "g2o/core/base_unary_edge.h"

#include "g2o/core/base_unary_edge.h"
#include "g2o/types/sba/types_six_dof_expmap.h"

#include "LibStevi/geometry/core.h"

namespace StereoVisionApp {

/*!
 * \brief The EdgeXyzPrior class allow to get an apriori for the position of a landmark
 */
class EdgeXyzPrior : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexPointXYZ>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EdgeXyzPrior();

	virtual bool read(std::istream& is);
	virtual bool write(std::ostream& os) const;

	virtual void computeError();
};

/*!
 * \brief The EdgeXyzPrior class allow to get an apriori for the position of a landmark
 */
template<StereoVision::Geometry::Axis Ax>
class EdgeLmCoordPrior : public g2o::BaseUnaryEdge<1, double, g2o::VertexPointXYZ>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EdgeLmCoordPrior() {

	}

	virtual bool read(std::istream& is) {
		double p;
		is >> p;
		setMeasurement(p);
		is >> information()(0, 0);
		return true;
	}
	virtual bool write(std::ostream& os) const {
		os << measurement() << " ";
		os << " " << information()(0, 0);
		return os.good();
	}

	virtual void computeError() {

		const g2o::VertexPointXYZ* v = static_cast<const g2o::VertexPointXYZ*>(_vertices[0]);

		int id;

		if (Ax == StereoVision::Geometry::Axis::X) {
			id = 0;
		}

		if (Ax == StereoVision::Geometry::Axis::Y) {
			id = 1;
		}

		if (Ax == StereoVision::Geometry::Axis::Z) {
			id = 2;
		}

		_error[0] = v->estimate()[id] - _measurement;
	}
};

} // namespace StereoVisionApp

#endif // EDGEXYZPRIOR_H
