#ifndef FIXEDSIZENORMALPRIOR_H
#define FIXEDSIZENORMALPRIOR_H

#include <ceres/cost_function.h>
#include <Eigen/Core>

namespace StereoVisionApp {

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
