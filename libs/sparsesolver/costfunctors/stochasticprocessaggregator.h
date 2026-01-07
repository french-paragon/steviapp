#ifndef STOCHASTICPROCESSAGGREGATOR_H
#define STOCHASTICPROCESSAGGREGATOR_H

#include "../../utils/functional.h"

#include <array>
#include <vector>
#include <set>
#include <map>

#include <QDebug>

namespace StereoVisionApp {

/*!
 * \brief The StochasticProcessAggregator class aggregate different parameters to a single parameter as a sum.
 *
 * This class is optimized for aggregating a bunch of parameters at once, possibly targeting multiple parameters of the inner functor.
 * This is usefull when modelling complex stochastic processes, like e.g., the bias and scale factors for an inertial system.
 */
template<typename FunctorT, bool isDynamicFunctor, int paramTargetDims, int subFunctorNParams = -1>
class StochasticProcessAggregator : private FunctorT, public virtual BaseDynamicDecorator {

    static_assert(isDynamicFunctor or subFunctorNParams > 0, "If the decorated functor is not dynamic, then the number of parameters of the functor must be given as template argument");

public:

    template <typename ... P>
    StochasticProcessAggregator(P... args) :
        FunctorT(args...)
    {

    }

    void setAccumulationWeights(std::vector<int> const& paramsMap, std::vector<double> const& weights) {

        if (paramsMap.size() != weights.size()) {
            return;
        }

        _paramMaps = paramsMap;

        std::set<int> viewedIdxs;

        for (int idx : paramsMap) {
            viewedIdxs.insert(idx);
        }

        _inverseParamMaps.resize(viewedIdxs.size());
        std::copy(viewedIdxs.begin(), viewedIdxs.end(), _inverseParamMaps.begin());

        std::map<int,int> matching;

        for (int i = 0; i < _inverseParamMaps.size(); i++) {
            matching[_inverseParamMaps[i]] = i;
        }

        for (int i = 0; i < _paramMaps.size(); i++) {
            _paramMaps[i] = matching[_paramMaps[i]];
        }

        _accumulation_weights = weights;
    }

    inline void setAccumulationWeights(int accumulationParamIdx, std::vector<double> const& weights) {
        std::vector<int> idxs(weights.size());
        std::fill(idxs.begin(), idxs.end(), accumulationParamIdx);
        setAccumulationWeights(idxs, weights);
    }

    template <typename T, typename ... P>
    bool operator()(T const* const* parameters, T* residuals) const {


        int currentNParams = nParams();
        int nextNParams = currentNParams - _accumulation_weights.size();

        if constexpr (!isDynamicFunctor) { //static functor
            if (nextNParams != subFunctorNParams) {
                return false;
            }
        } else {
            if (nextNParams <= 0) {
                return false;
            }
        }

        std::vector<std::array<T,paramTargetDims>> accumulated(_inverseParamMaps.size());

        for (int i = 0; i < _inverseParamMaps.size(); i++) {
            for (int j = 0; j < paramTargetDims; j++) {
                accumulated[i][j] = parameters[_inverseParamMaps[i]][j];
            }
        }

        for (int i = nextNParams; i < currentNParams; i++) {
            int wIdx = i - nextNParams;
            for (int j = 0; j < paramTargetDims; j++) {
                T delta = _accumulation_weights[wIdx]*parameters[i][j];
                accumulated[_paramMaps[wIdx]][j] += delta;
            }
        }

        if constexpr (isDynamicFunctor) {

            std::vector<T const*> processedArgs(nextNParams);

            for (int i = 0; i < nextNParams; i++) {
                processedArgs[i] = parameters[i];
            }

            for (int i = 0; i < _inverseParamMaps.size(); i++) {
                processedArgs[_inverseParamMaps[i]] = accumulated[i].data();
            }

            setNParams(nextNParams);

            bool ok = FunctorT::operator()(processedArgs.data(), residuals);

            //reset the correct number of parameters
            setNParams(currentNParams);

            if (!ok) {
                return false;
            }

        } else {

            std::array<T const*, subFunctorNParams> processedArgs;

            for (int i = 0; i < subFunctorNParams; i++) {
                processedArgs[i] = parameters[i];
            }

            for (int i = 0; i < _inverseParamMaps.size(); i++) {
                processedArgs[_inverseParamMaps[i]] = accumulated[i].data();
            }

            std::tuple<T*> resTuple{residuals};

            auto processedArgsTuple = std::tuple_cat(ArrayToTuple<T const*, subFunctorNParams>::convert(processedArgs), resTuple);


            auto variadic_lambda = [this] (auto... params) {
                (void) this;
                return FunctorT::operator()(params...);
            };

            return CallFromTuple::call(variadic_lambda, processedArgsTuple);

        }

        return true;

    }

private:

    std::vector<int> _paramMaps;
    std::vector<int> _inverseParamMaps;
    std::vector<double> _accumulation_weights;
};

/*!
 * \brief The StationaryGaussMarkovProcessCost class represent an error function for two successive nodes in a Stationary Gauss Markov (or Ornstein–Uhlenbeck) process.
 */
template <int nDim>
class StationaryGaussMarkovProcessCost
{
public:
    StationaryGaussMarkovProcessCost(std::array<double,nDim> const& betas,
                                     std::array<double,nDim> const& sigmas,
                                     std::array<double,nDim> const& means,
                                     double dt) :
        _betas(betas),
        _sigmas(sigmas),
        _means(means),
        _dt(dt)
    {

    }
    StationaryGaussMarkovProcessCost(std::array<double,nDim> const& betas,
                                     std::array<double,nDim> const& sigmas,
                                     double dt) :
        _betas(betas),
        _sigmas(sigmas),
        _dt(dt)
    {
        std::fill(_means.begin(), _means.end(), 0);
    }
    StationaryGaussMarkovProcessCost(std::array<double,nDim> const& betas,
                                     double dt) :
        _betas(betas),
        _dt(dt)
    {
        std::fill(_means.begin(), _means.end(), 0);
        std::fill(_sigmas.begin(), _sigmas.end(), 1);
    }

    template <typename T>
    bool operator()(const T* const prev,
                    const T* const next,
                    T* residual) const {

        for (int i = 0; i < nDim; i++) {
            T scale = T(exp(-_betas[i]*abs(_dt)));
            residual[i] = (next[i] - scale*prev[i] - T(_means[i])*(T(1)-scale))/T(_sigmas[i]); //expected values
        }

        return true;
    }

protected:

    std::array<double,nDim> _betas;
    std::array<double,nDim> _sigmas;
    std::array<double,nDim> _means;
    double _dt;
};

}

#endif // STOCHASTICPROCESSAGGREGATOR_H
