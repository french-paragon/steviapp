#include "lensdistortion.h"

namespace StereoVisionApp {

Eigen::Vector2f radialDistortion(Eigen::Vector2f pos, Eigen::Vector3f k123) {

	float r2 = pos(0)*pos(0) + pos(1)*pos(1);

	Eigen::Vector3f vr;
	vr << r2, r2*r2, r2*r2*r2;

	return k123.dot(vr)*pos;
}

Eigen::Vector2d radialDistortionD(Eigen::Vector2d pos, Eigen::Vector3d k123) {

	double r2 = pos(0)*pos(0) + pos(1)*pos(1);

	Eigen::Vector3d vr;
	vr << r2, r2*r2, r2*r2*r2;

	return k123.dot(vr)*pos;
}

Eigen::Vector2f tangentialDistortion(Eigen::Vector2f pos, Eigen::Vector2f t12) {

	float r2 = pos(0)*pos(0) + pos(1)*pos(1);

	Eigen::Vector2f td;
	td << t12(1)*(r2 + 2*pos(0)*pos(0)) + 2*t12(0)*pos(0)*pos(1),
			t12(0)*(r2 + 2*pos(1)*pos(1)) + 2*t12(1)*pos(0)*pos(1);

	return td;

}
Eigen::Vector2d tangentialDistortionD(Eigen::Vector2d pos, Eigen::Vector2d t12) {

	double r2 = pos(0)*pos(0) + pos(1)*pos(1);

	Eigen::Vector2d td;
	td << t12(1)*(r2 + 2*pos(0)*pos(0)) + 2*t12(0)*pos(0)*pos(1),
			t12(0)*(r2 + 2*pos(1)*pos(1)) + 2*t12(1)*pos(0)*pos(1);

	return td;
}

Eigen::Vector2f skewDistortion(Eigen::Vector2f pos, Eigen::Vector2f B12, float f, Eigen::Vector2f pp) {

	Eigen::Vector2f r = f*pos + pp;
	r[0] += B12[0]*pos[0] + B12[1]*pos[1];
	return r;
}
Eigen::Vector2d skewDistortionD(Eigen::Vector2d pos, Eigen::Vector2d B12, double f, Eigen::Vector2d pp) {

	Eigen::Vector2d r = f*pos + pp;
	r[0] += B12[0]*pos[0] + B12[1]*pos[1];
	return r;
}

} // namespace StereoVisionApp
