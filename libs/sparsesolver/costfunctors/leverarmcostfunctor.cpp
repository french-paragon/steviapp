#include "leverarmcostfunctor.h"

namespace StereoVisionApp {

LeverArmCostFunctor::LeverArmCostFunctor(Eigen::Matrix3d const& R, Eigen::Vector3d const& t):
    _R(R),
    _t(t)
{

}

ParametrizedLeverArmCostFunctor::ParametrizedLeverArmCostFunctor()
{

}

InterpolatedLeverArmCostFunctor::InterpolatedLeverArmCostFunctor(Eigen::Matrix3d const& R,
                                                                 Eigen::Vector3d const& t,
                                                                 double w1,
                                                                 double w2) :
    _R(R),
    _t(t),
    _w1(w1),
    _w2(w2)
{

}

ParametrizedInterpolatedLeverArmCostFunctor::ParametrizedInterpolatedLeverArmCostFunctor(double w1,
                                                                                         double w2) :
    _w1(w1),
    _w2(w2)
{

}

} // namespace StereoVisionApp
