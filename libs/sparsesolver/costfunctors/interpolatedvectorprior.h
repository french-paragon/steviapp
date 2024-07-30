#ifndef STEREOVISIONAPP_INTERPOLATEDVECTORPRIOR_H
#define STEREOVISIONAPP_INTERPOLATEDVECTORPRIOR_H

#include <Eigen/Core>

#ifndef NDEBUG
#include <iostream>
#include <ceres/jet.h>
#endif

namespace StereoVisionApp {

/*!
 * \brief The InterpolatedVectorPrior class represent a prior on the linearily interpolated value between two vectors
 */
template<int dim>
class InterpolatedVectorPrior
{
public:

    using VecT = Eigen::Matrix<double,dim,1>;
    using MatInfosT = Eigen::Matrix<double,dim,dim>;
    using WeightT = double;

    InterpolatedVectorPrior(VecT const& measurement, WeightT w1, WeightT w2, MatInfosT const& infos = MatInfosT::Identity()) :
        _infos(infos),
        _prior(measurement),
        _w1(w1),
        _w2(w2)
    {

    }

    template <typename T>
    bool operator()(const T* const v1, const T* const v2, T* residual) const {

        using Vec = Eigen::Matrix<T,dim,1>;
        using MatInfos = Eigen::Matrix<T,dim,dim>;

        Vec vec1;
        Vec vec2;

        for (int i = 0; i < dim; i++) {
            vec1[i] = v1[i];
            vec2[i] = v2[i];
        }

        Vec interpolated = _w1*vec1 + _w2*vec2;

        Vec error = interpolated - _prior;

        Vec wError;

        for (int i = 0; i < dim; i++) {
            wError[i] = T(0);

            for (int j = 0; j < dim; j++) {
                wError[i] += _infos(i,j)*error[j];
            }
        }


        for (int i = 0; i < dim; i++) {
            residual[i] = wError[i];
        }

#ifndef NDEBUG
        for (int i = 0; i < dim; i++) {
            if (!ceres::IsFinite(residual[i])) {
                std::cout << "Error in InterpolatedVectorPrior<" << dim << "> cost computation" << std::endl;
            }
        }
#endif

        return true;
    }

protected:

    MatInfosT _infos;
    VecT _prior;
    WeightT _w1, _w2;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_INTERPOLATEDVECTORPRIOR_H
