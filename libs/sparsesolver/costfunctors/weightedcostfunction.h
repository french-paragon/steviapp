#ifndef STEREOVISIONAPP_WEIGTHEDCOSTFUNCTION_H
#define STEREOVISIONAPP_WEIGTHEDCOSTFUNCTION_H

#include <ceres/sized_cost_function.h>

#include <stdint.h>

#include <Eigen/Core>

#ifndef NDEBUG
#include <iostream>
#include <ceres/jet.h>
#endif

namespace StereoVisionApp {

template<int kNumResiduals, int... Ns>
class WeightedCostFunction : public ceres::SizedCostFunction<kNumResiduals, Ns...>
{
public:

    using WeigthMatT = Eigen::Matrix<double, kNumResiduals, kNumResiduals>;
    using ResVecT = Eigen::Matrix<double, kNumResiduals, 1>;
    using JacMatT = Eigen::Matrix<double, kNumResiduals, Eigen::Dynamic>;

    /*!
     * \brief WeigthedCostFunction build a weigthed cost function from an unweigthed one.
     * \param unweigthedFunction a pointer to the unweigthed cost function (the weigthed function take ownership of it).
     * \param sqrtWeight the square root of the weight matrix.
     */
    WeightedCostFunction(ceres::SizedCostFunction<kNumResiduals, Ns...>* unweigthedFunction, WeigthMatT const& sqrtWeight) :
        _unweigthedCostFunction(unweigthedFunction),
        _sqrtWeight(sqrtWeight)
    {

    }

    virtual ~WeightedCostFunction() {
        if (_unweigthedCostFunction != nullptr) {
            delete _unweigthedCostFunction;
        }
    }

    virtual bool Evaluate(double const* const* parameters,
                          double* residuals,
                          double** jacobians) const override {

        bool ok = _unweigthedCostFunction->Evaluate(parameters, residuals, jacobians);

        if (!ok) {
            return false;
        }

        ResVecT res;

        for (int i = 0; i < kNumResiduals; i++) {
            res[i] = residuals[i];
        }

        ResVecT w_res = _sqrtWeight*res;

        for (int i = 0; i < kNumResiduals; i++) {
            residuals[i] = w_res[i];
        }

        if (jacobians == nullptr) {
            return true;
        }

        std::array<int, sizeof... (Ns)> block_sizes = {Ns...};

        for (int i = 0; i < sizeof... (Ns); i++) {

            if (jacobians[i] == nullptr) { //parameter is constant
                continue;
            }

            JacMatT J;
            J.resize(kNumResiduals, block_sizes[i]);

            for (int r = 0; r < kNumResiduals; r++) {
                for (int c = 0; c < block_sizes[i]; c++) {
                    J(r,c) = jacobians[i][r * block_sizes[i] + c];
                }
            }

            JacMatT wJ = _sqrtWeight*J;

            for (int r = 0; r < kNumResiduals; r++) {
                for (int c = 0; c < block_sizes[i]; c++) {
                    jacobians[i][r * block_sizes[i] + c] = wJ(r,c);
                }
            }

        }

        return true;


    }

protected:

    ceres::SizedCostFunction<kNumResiduals, Ns...>* _unweigthedCostFunction;
    Eigen::Matrix<double, kNumResiduals, kNumResiduals> _sqrtWeight;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_WEIGTHEDCOSTFUNCTION_H
