#ifndef VERTEXCAMERATANGENTIALDISTORSION_H
#define VERTEXCAMERATANGENTIALDISTORSION_H

#include <Eigen/Core>
#include "g2o/core/base_vertex.h"

/*!
 * \brief The VertexCameraTangentialDistorsion class represent the two tangential distorsion parameters of a lens (T1 and T2)
 */
class VertexCameraTangentialDistortion : public g2o::BaseVertex<2, Eigen::Vector2d>
{
public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	VertexCameraTangentialDistortion();

	virtual bool read(std::istream& is);

	virtual bool write(std::ostream& os) const;

	virtual void setToOriginImpl();

	virtual void oplusImpl(const double* update_);
};

#endif // VERTEXCAMERATANGENTIALDISTORSION_H
