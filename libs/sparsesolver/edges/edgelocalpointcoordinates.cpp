#include "edgelocalpointcoordinates.h"

#include <QDebug>

namespace StereoVisionApp {

EdgeLocalPointCoordinates::EdgeLocalPointCoordinates()
{

}

bool EdgeLocalPointCoordinates::read(std::istream& is) {
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
bool EdgeLocalPointCoordinates::write(std::ostream& os) const {
	Eigen::Vector3d p = measurement();
	os << p[0] << " " << p[1] << " " << p[2] << " ";
	for (int i = 0; i < 3; ++i)
		for (int j = i; j < 3; ++j)
			os << " " << information()(i, j);
	return os.good();
}

void EdgeLocalPointCoordinates::computeError() {

	const VertexXiType* v1 = static_cast<const VertexXiType*>(_vertices[0]);
	VertexXiType::EstimateType localToWorld = v1->estimate();
	const VertexXjType* v2 = static_cast<const VertexXjType*>(_vertices[1]);
	VertexXjType::EstimateType ptWorldPos = v2->estimate();

	Eigen::Vector3d ptLocalPos = measurement();

	_error = localToWorld.rotation()*ptLocalPos + localToWorld.translation() - ptWorldPos;

	if (_error.hasNaN()) {
		qDebug() << "EdgeLocalPointCoordinates";
	}
}

} // namespace StereoVisionApp
