#include "edgese3rpyprior.h"

namespace StereoVisionApp {

EdgeSE3rpyPrior::EdgeSE3rpyPrior() : g2o::BaseUnaryEdge<3, Eigen::Matrix3d, VertexCameraPose>()
{

}

bool EdgeSE3rpyPrior::read(std::istream& is) {
	Eigen::AngleAxisd a;
	is >> a.axis()[0] >> a.axis()[1] >> a.axis()[2];

	Measurement q = a.toRotationMatrix();

	setMeasurement(q);
	for (int i = 0; i < 3; ++i)
	  for (int j = i; j < 3; ++j) {
		is >> information()(i, j);
		if (i != j)
		  information()(j, i) = information()(i, j);
	  }
	return true;
}

bool EdgeSE3rpyPrior::write(std::ostream& os) const {

	Measurement q = measurement();
	Eigen::AngleAxisd a(q);
	os << a.axis()[0] << " " << a.axis()[1] << " " << a.axis()[1];
	for (int i = 0; i < 3; ++i)
	  for (int j = i; j < 3; ++j)
		os << " " << information()(i, j);
	return os.good();
}

void EdgeSE3rpyPrior::computeError()
{
	const VertexXiType* v = static_cast<const VertexXiType*>(_vertices[0]);
	VertexXiType::EstimateType pose = v->estimate();
	Eigen::AngleAxisd r(pose.rotation().inverse() * _measurement);
	_error = r.axis();

}

} // namespace StereoVisionApp
