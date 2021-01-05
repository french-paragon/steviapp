#include "edgese3xyzprior.h"

namespace StereoVisionApp {

EdgeSE3xyzPrior::EdgeSE3xyzPrior() : g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexCameraPose>()
{

}

bool EdgeSE3xyzPrior::read(std::istream& is) {
	Eigen::Vector3d p;
	is >> p[0] >> p[1] >> p[2];
	setMeasurement(p);
	for (int i = 0; i < 3; ++i)
	  for (int j = i; j < 3; ++j) {
		is >> information()(i, j);
		if (i != j)
		  information()(j, i) = information()(i, j);
	  }
	return true;
}

bool EdgeSE3xyzPrior::write(std::ostream& os) const {
	Eigen::Vector3d p = measurement();
	os << p[0] << " " << p[1] << " " << p[1];
	for (int i = 0; i < 3; ++i)
	  for (int j = i; j < 3; ++j)
		os << " " << information()(i, j);
	return os.good();
}

void EdgeSE3xyzPrior::computeError()
{
  const VertexXiType* v = static_cast<const VertexXiType*>(_vertices[0]);
  VertexXiType::EstimateType pose = v->estimate();
  _error = pose.translation() - _measurement;
}

} // namespace StereoVisionApp
