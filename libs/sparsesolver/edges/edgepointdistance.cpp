#include "edgepointdistance.h"

#include <QDebug>

namespace StereoVisionApp {

EdgePointDistance::EdgePointDistance()
{

}

bool EdgePointDistance::read(std::istream& is) {
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
bool EdgePointDistance::write(std::ostream& os) const {
	Measurement q = measurement();
	os << q << " ";
	for (int i = 0; i < Dimension; ++i)
		for (int j = i; j < Dimension; ++j)
			os << " " << information()(i, j);
	return os.good();
}

void EdgePointDistance::computeError() {
	const VertexXiType* v1 = static_cast<const VertexXiType*>(_vertices[0]);
	VertexXiType::EstimateType pt1 = v1->estimate();
	const VertexXjType* v2 = static_cast<const VertexXjType*>(_vertices[1]);
	VertexXjType::EstimateType pt2 = v2->estimate();

	double dist = (pt1 - pt2).norm();

	_error[0] = dist - _measurement;

	if (_error.hasNaN()) {
		qDebug() << "EdgePointDistance";
	}

}

} // namespace StereoVisionApp
