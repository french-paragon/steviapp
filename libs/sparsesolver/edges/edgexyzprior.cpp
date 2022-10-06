#include "edgexyzprior.h"

namespace StereoVisionApp {

EdgeXyzPrior::EdgeXyzPrior() : g2o::BaseUnaryEdge< 3, Eigen::Vector3d, g2o::VertexPointXYZ >()
{

}

bool EdgeXyzPrior::read(std::istream& is) {
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

bool EdgeXyzPrior::write(std::ostream& os) const {
	Eigen::Vector3d p = measurement();
	os << p[0] << " " << p[1] << " " << p[2] << " ";
	for (int i = 0; i < 3; ++i)
	  for (int j = i; j < 3; ++j)
		os << " " << information()(i, j);
	return os.good();
}

void EdgeXyzPrior::computeError()
{
  const g2o::VertexPointXYZ* v = static_cast<const g2o::VertexPointXYZ*>(_vertices[0]);
  _error = v->estimate() - _measurement;

}

} // namespace StereoVisionApp
