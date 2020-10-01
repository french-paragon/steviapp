#include "edgese3fullprior.h"

EdgeSE3FullPrior::EdgeSE3FullPrior() : g2o::BaseUnaryEdge<6, CameraPose, VertexCameraPose>()
{

}

bool EdgeSE3FullPrior::read(std::istream& is) {
	Eigen::Vector3d p;
	is >> p[0] >> p[1] >> p[2];
	Eigen::AngleAxisd a;
	is >> a.axis()[0] >> a.axis()[1] >> a.axis()[2];
	Measurement q(Eigen::Quaterniond(a), p);
	setMeasurement(q);
	for (int i = 0; i < 6; ++i)
		for (int j = i; j < 6; ++j) {
			is >> information()(i, j);
			if (i != j)
				information()(j, i) = information()(i, j);
		}
	return true;
}

bool EdgeSE3FullPrior::write(std::ostream& os) const {
	Measurement q = measurement();
	Eigen::Vector3d p = q.translation();
	Eigen::AngleAxisd a(q.rotation());
	os << p[0] << " " << p[1] << " " << p[2] << " ";
	os << a.axis()[0] << " " << a.axis()[1] << " " << a.axis()[1];
	for (int i = 0; i < 6; ++i)
		for (int j = i; j < 6; ++j)
			os << " " << information()(i, j);
	return os.good();
}

void EdgeSE3FullPrior::computeError()
{
	const VertexXiType* v = static_cast<const VertexXiType*>(_vertices[0]);
	VertexXiType::EstimateType pose = v->estimate();
	VertexXiType::EstimateType q = pose.inverse() * _measurement;
	_error = q.log(); //implementation using Rodriguez formula.
}
