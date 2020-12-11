#include "imagecoordinates.h"

namespace StereoVisionApp {


Eigen::Vector2f Image2HomogeneousCoordinates(Eigen::Vector2f const& pt,
											 float fx,
											 float fy,
											 Eigen::Vector2f const& pp,
											 ImageAnchors imageOrigin) {

	Eigen::Vector2f r = pt - pp;
	r.x() /= fx;
	r.y() /= fy;

	switch (imageOrigin) {
	case ImageAnchors::TopLeft:
		r.y() = -r.y();
		break;
	case ImageAnchors::TopRight:
		r.x() = -r.x();
		r.y() = -r.y();
		break;
	case ImageAnchors::BottomLeft:
	case ImageAnchors::BottomRight:
		r.x() = -r.x();
		break;
	}

	return r;

}

Eigen::Vector2f Image2HomogeneousCoordinates(Eigen::Vector2f const& pt,
											 float f,
											 Eigen::Vector2f const& pp,
											 ImageAnchors imageOrigin) {

	return Image2HomogeneousCoordinates(pt, f, f, pp, imageOrigin);

}


Eigen::Array2Xf Image2HomogeneousCoordinates(Eigen::Array2Xf const& pt,
											 float fx,
											 float fy,
											 Eigen::Vector2f const& pp,
											 ImageAnchors imageOrigin) {

	Eigen::Array2Xf r = pt;
	r.row(0) -= pp.x();
	r.row(1) -= pp.y();

	r.row(0) /= fx;
	r.row(1) /= fy;

	switch (imageOrigin) {
	case ImageAnchors::TopLeft:
		r.row(1) *= -1;
		break;
	case ImageAnchors::TopRight:
		r.row(0) *= -1;
		r.row(1) *= -1;
		break;
	case ImageAnchors::BottomLeft:
	case ImageAnchors::BottomRight:
		r.row(0) *= -1;
		break;
	}

	return r;

}

Eigen::Array2Xf Image2HomogeneousCoordinates(Eigen::Array2Xf const& pt,
											 float f,
											 Eigen::Vector2f const& pp,
											 ImageAnchors imageOrigin) {
	return Image2HomogeneousCoordinates(pt, f, f, pp, imageOrigin);
}

} // namespace StereoVisionApp
