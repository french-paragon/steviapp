#ifndef VERTEXCAMERASKEWDISTORSTION_H
#define VERTEXCAMERASKEWDISTORSTION_H

#include <Eigen/Core>
#include "g2o/core/base_vertex.h"

namespace StereoVisionApp {

class VertexCameraSkewDistortion : public g2o::BaseVertex<2, Eigen::Vector2d>
{
public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	VertexCameraSkewDistortion();

	virtual bool read(std::istream& is);

	virtual bool write(std::ostream& os) const;

	virtual void setToOriginImpl();

	virtual void oplusImpl(const double* update_);
};

} // namespace StereoVisionApp

#endif // VERTEXCAMERASKEWDISTORSTION_H
