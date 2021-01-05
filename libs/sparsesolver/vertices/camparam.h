#ifndef CAMPARAM_H
#define CAMPARAM_H

#include <eigen3/Eigen/Core>
#include <vector>

namespace StereoVisionApp {

/*!
 * \brief The CamParam class is a storage class for the VertexCameraParam (\see VertexCameraParam) class.
 */
class CamParam
{
public:
	CamParam( Eigen::Vector2d const& extend = Eigen::Vector2d::Zero(),
			  Eigen::Vector2d const& pp = Eigen::Vector2d::Zero(),
			  double const& f = 1);

	friend std::istream & operator>> (std::istream & in, CamParam & cam);
	friend std::ostream & operator<< (std::ostream & out, CamParam const& cam);

	CamParam& operator+= (CamParam const& incr);
	CamParam& operator+= (std::vector<double> const& incr);
	CamParam& operator+= (double const* incr);

	Eigen::Vector2d operator* (Eigen::Vector3d const& point) const;

	Eigen::Vector2d extend() const;
	void setExtend(const Eigen::Vector2d &extend);

	Eigen::Vector2d pp() const;
	void setPp(const Eigen::Vector2d &pp);

	double f() const;
	void setF(double f);

protected:

	Eigen::Vector2d _extend;
	Eigen::Vector2d _pp;
	double _f;
};

std::istream & operator>> (std::istream & in, CamParam & cam);
std::ostream & operator<< (std::ostream & out, CamParam const& cam);

} // namespace StereoVisionApp

#endif // CAMPARAM_H
