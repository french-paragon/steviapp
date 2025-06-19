#ifndef FIXEDSIZENORMALPRIOR_H
#define FIXEDSIZENORMALPRIOR_H

#include <ceres/cost_function.h>
#include <Eigen/Core>

#ifndef NDEBUG
#include <iostream>
#include <ceres/jet.h>
#endif

namespace StereoVisionApp {

/*!
 * \brief The FixedSizeNormalCostFunctor class is a functor version of FixedSizeNormalPrior
 *
 * It does not use the trivial formula to compute the jacobian, but can be more easily modified using decorators.
 */
template<int in, int out>
class FixedSizeNormalCostFunctor {
public:

    using MatrixT = Eigen::Matrix<double,out,in>;
    using VectorT = Eigen::Matrix<double,in,1>;

    FixedSizeNormalCostFunctor(MatrixT const& A, VectorT const& b) :
        _A(A),
        _b(b)
    {
    }
    template <typename T>
    bool operator()(const T* const x, T* residuals) const {

        using VecIn = Eigen::Matrix<T,in,1>;
        using VecOut = Eigen::Matrix<T,out,1>;
        using Mat = Eigen::Matrix<T,out,in>;

        VecIn vx;
        for (int i = 0; i < in; i++) {
            vx[i] = x[i];
        }

        VecOut r = _A.template cast<T>() *(vx - _b.template cast<T>());

        for (int i = 0; i < out; i++) {
            residuals[i] = r[i];
        }

#ifndef NDEBUG
        for (int i = 0; i < out; i++) {
            if (!ceres::isfinite(residuals[i])) {
                std::cout << "Error in InterpolatedVectorPrior<" << in << "," << out << "> cost computation" << std::endl;
            }
        }
#endif

        return true;

    }

private:
    MatrixT _A;
    VectorT _b;
};

/*!
 * \brief The FixedSizeNormalPrior class is a replacement for ceres normal prior, based on templated eigen types rather than dynamic sized matrices.
 */
template<int in, int out>
class FixedSizeNormalPrior : public ceres::CostFunction {
public:

    using MatrixT = Eigen::Matrix<double,out,in>;
    using VectorT = Eigen::Matrix<double,in,1>;
    using ResultT = Eigen::Matrix<double,out,1>;

    FixedSizeNormalPrior(MatrixT const& A, VectorT const& b) :
        _A(A),
        _b(b)
    {
        set_num_residuals(out);
        mutable_parameter_block_sizes()->push_back(in);
    }

    bool Evaluate(double const* const* parameters,
                  double* residuals,
                  double** jacobians) const override {

        VectorT p;

        for (int i = 0; i < in; i++) {
            p[i] = parameters[0][i];
        }

        ResultT r = _A*(p - _b);

        for (int i = 0; i < out; i++) {
            residuals[i] = r[i];
        }

        if ((jacobians != nullptr) && (jacobians[0] != nullptr)) {
            for (int i = 0; i < out; i++) {
                for (int j = 0; j < in; j++) {
                    jacobians[0][i * in + j] = _A(i,j);
                }
            }
        }
        return true;
    }

private:
    MatrixT _A;
    VectorT _b;
};

}
#endif // FIXEDSIZENORMALPRIOR_H
