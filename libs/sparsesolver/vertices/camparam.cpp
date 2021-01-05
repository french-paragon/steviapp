#include "camparam.h"

namespace StereoVisionApp {

CamParam::CamParam(const Eigen::Vector2d &extend,
				   const Eigen::Vector2d &pp,
				   const double &f) :
	_extend(extend),
	_pp(pp),
	_f(f)
{

}

CamParam& CamParam::operator+= (CamParam const& incr) {
	_pp += incr.pp();
	_f += incr.f();
	return *this;
}

CamParam& CamParam::operator+= (std::vector<double> const& incr) {

	Eigen::Vector2d d;
	d << incr[0], incr[1];
	_pp += d;
	_f += incr[2];
	return *this;
}
CamParam& CamParam::operator+= (double const* incr) {

	Eigen::Vector2d d;
	d << incr[0], incr[1];
	_pp += d;
	_f += incr[2];
	return *this;
}

Eigen::Vector2d CamParam::operator* (Eigen::Vector3d const& point) const{

	return _f/point(2)*point.block<2, 1>(0, 0) + _pp;

}

Eigen::Vector2d CamParam::extend() const
{
	return _extend;
}

void CamParam::setExtend(const Eigen::Vector2d &extend)
{
	_extend = extend;
}

Eigen::Vector2d CamParam::pp() const
{
	return _pp;
}

void CamParam::setPp(const Eigen::Vector2d &pp)
{
	_pp = pp;
}

double CamParam::f() const
{
	return _f;
}

void CamParam::setF(double f)
{
	_f = f;
}

std::istream & operator>> (std::istream & in, CamParam & cam) {
	Eigen::Vector2d extend;
	Eigen::Vector2d pp;
	double f;

	in >> extend(0);
	in >> extend(1);

	in >> pp(0);
	in >> pp(1);

	in >> f;

	cam.setExtend(extend);
	cam.setPp(pp);
	cam.setF(f);

	return in;
}

std::ostream & operator<< (std::ostream & out, CamParam const& cam) {
	out << cam.extend()(0) << ' ';
	out << cam.extend()(1) << ' ';

	out << cam.pp()(0) << ' ';
	out << cam.pp()(1) << ' ';

	out << cam.f();

	return out;
}

} // namespace StereoVisionApp
