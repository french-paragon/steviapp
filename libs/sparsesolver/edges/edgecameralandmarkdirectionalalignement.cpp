#include "edgecameralandmarkdirectionalalignement.h"

namespace StereoVisionApp {

MinCamDistanceParam::MinCamDistanceParam() {

}

MinCamDistanceParam::MinCamDistanceParam(double pMinDist):
	minDist(pMinDist)
{

}

double MinCamDistanceParam::minCameraDistance () const {
	return minDist;
}

bool MinCamDistanceParam::read (std::istream& is){
  is >> minDist;
  return true;
}

bool MinCamDistanceParam::write (std::ostream& os) const {
  os << minDist << " ";
  return true;
}

EdgeCameraLandmarkDirectionalAlignement::EdgeCameraLandmarkDirectionalAlignement()
{
	_minCamDistParam = 0;
	resizeParameters(1);
	installParameter(_minCamDistParam, 0);
}

bool EdgeCameraLandmarkDirectionalAlignement::read(std::istream& is) {
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
bool EdgeCameraLandmarkDirectionalAlignement::write(std::ostream& os) const {
	Eigen::Vector3d p = measurement();
	os << p[0] << " " << p[1] << " " << p[2] << " ";
	for (int i = 0; i < 3; ++i)
		for (int j = i; j < 3; ++j)
			os << " " << information()(i, j);
	return os.good();
}

void EdgeCameraLandmarkDirectionalAlignement::setMeasurementFromNormalizedCoordinates(Eigen::Vector2d const& normalized) {
	Eigen::Vector3d mes(normalized[0], normalized[1], 1);
	mes.normalize();
	setMeasurement(mes);
}

void EdgeCameraLandmarkDirectionalAlignement::computeError() {

	const VertexXiType* v1 = static_cast<const VertexXiType*>(_vertices[0]);
	VertexXiType::EstimateType campose = v1->estimate();
	const VertexXjType* v2 = static_cast<const VertexXjType*>(_vertices[1]);
	VertexXjType::EstimateType point = v2->estimate();

	const MinCamDistanceParam* p = static_cast<const MinCamDistanceParam*>(parameter(0));
	double minDist = 1;

	if (p != nullptr) {
		p->minCameraDistance();
	}
	\
	Eigen::Vector3d Pbar = campose.r().transpose()*(point - campose.t());
	Pbar *= minDist;

	double norm = Pbar.norm();

	if (norm > 1) {
		Pbar /= norm;
	}

	Pbar /= minDist;

	_error = Pbar - _measurement;
}

} // namespace StereoVisionApp
