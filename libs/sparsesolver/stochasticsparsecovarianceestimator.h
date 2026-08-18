#ifndef STOCHASTICSPARSECOVARIANCEESTIMATOR_H
#define STOCHASTICSPARSECOVARIANCEESTIMATOR_H

#include <Eigen/Sparse>

#include <set>
#include <random>

namespace StereoVisionApp {

template<typename JacMatT, typename SolverT, typename HMatT = Eigen::SparseMatrix<double>,  typename RandomEngineT = std::default_random_engine, bool Robust = false>
class StochasticCovarianceFromJacobianBootstrapEstimator {

public:

    template<typename SolT>
    struct SolverTraits {
        static constexpr bool supported = false;
        static constexpr bool requireHessian = false;
        static constexpr bool IsIterative = false;
    };

    template<typename MatrixType_, int UpLo_, typename Preconditioner_>
    struct SolverTraits<Eigen::ConjugateGradient< MatrixType_, UpLo_, Preconditioner_ >> {
        static constexpr bool supported = true;
        static constexpr bool requireHessian = true;
        static constexpr bool IsIterative = true;
    };

    template<typename MatrixType_, typename Preconditioner_>
    struct SolverTraits<Eigen::LeastSquaresConjugateGradient<MatrixType_, Preconditioner_ >> {
        static constexpr bool supported = true;
        static constexpr bool requireHessian = false;
        static constexpr bool IsIterative = true;
    };

    template<typename MatrixType_, int UpLo_, typename Ordering_>
    struct SolverTraits<Eigen::SimplicialLLT< MatrixType_, UpLo_, Ordering_ >> {
        static constexpr bool supported = true;
        static constexpr bool requireHessian = true;
        static constexpr bool IsIterative = false;
    };

    template<typename MatrixType_, int UpLo_, typename Ordering_>
    struct SolverTraits<Eigen::SimplicialLDLT< MatrixType_, UpLo_, Ordering_ >> {
        static constexpr bool supported = true;
        static constexpr bool requireHessian = true;
        static constexpr bool IsIterative = false;
    };

    template<typename MatrixType_, typename Ordering_>
    struct SolverTraits<Eigen::SparseLU< MatrixType_, Ordering_ >> {
        static constexpr bool supported = true;
        static constexpr bool requireHessian = true;
        static constexpr bool IsIterative = false;
    };

    template<typename MatrixType_, typename Ordering_>
    struct SolverTraits<Eigen::SparseQR< MatrixType_, Ordering_ >> {
        static constexpr bool supported = true;
        static constexpr bool requireHessian = true;
        static constexpr bool IsIterative = false;
    };

    static_assert(SolverTraits<SolverT>::supported, "unsupported solver type!");

    struct Idx {
        int i;
        int j;
        inline bool operator<(Idx const& other) const {
            if (i == other.i) {
                return j < other.j;
            }
            return i < other.i;
        }
        inline bool operator==(Idx const& other) const {
            return i == other.i and  j == other.j;
        }
    };

    /*!
     * \brief StochasticCovarianceFromHessianEstimator build the  estimator from a solver
     * \param solver
     * \param vecSize
     * \param covariance_idxs
     */
    StochasticCovarianceFromJacobianBootstrapEstimator(JacMatT const& jacobian, std::vector<Idx> const& covariance_idxs) :
        _jacobian(jacobian),
        _normal_dist(0,1)
    {

        _n_obs = jacobian.rows();
        _n_params = jacobian.cols();
        _n_iterations = 0;

        int n = _n_params;
        std::set<Idx> inputIdxs;
        for (Idx const& idx : covariance_idxs) {
            if (idx.i < 0 or idx.i >= n) {
                continue;
            }
            if (idx.j < 0 or idx.j >= n) {
                continue;
            }

            if (idx.j < idx.i) {
                inputIdxs.insert(Idx{idx.j,idx.i});
            } else {
                inputIdxs.insert(idx);
            }
        }
        _target_idxs.resize(inputIdxs.size());
        std::copy(inputIdxs.begin(), inputIdxs.end(), _target_idxs.begin());
        std::sort(_target_idxs.begin(), _target_idxs.end());

        std::random_device rd;
        _re.seed(rd());

        if constexpr (SolverTraits<SolverT>::requireHessian) {
            _Hessian = _jacobian.transpose()*_jacobian;
            _solver.compute(_Hessian);
        } else {
            _solver.compute(_jacobian);
        }
    }

    void seed(int seed) {
        _re.seed(seed);
    }

    /*!
     * \brief statusOk check if the status is ok
     * \return true if status of solver is still Eigen::Success
     */
    inline bool statusOk() const {
        return _solver.info() == Eigen::Success;
    }

    inline int solverIterations() const {
        if constexpr (SolverTraits<SolverT>::IsIterative) {
            return _solver.iterations();
        } else {
            return 1;
        }
    }

    inline double solverError() const {
        if constexpr (SolverTraits<SolverT>::IsIterative) {
            return _solver.error();
        } else {
            return 0;
        }
    }

    /*!
     * \brief nIterations quick access to previous number of iterations
     * \return return the last number of iterations that was used for computation
     */
    inline int nIterations() const {
        return _n_iterations;
    }

    std::vector<Idx> const& targetIdxs() const {
        return _target_idxs;
    }

    Eigen::VectorXd computeEstimates(int nIterations) {

        int n = _n_obs;
        int p = _n_params;

        Eigen::VectorXd ret;
        ret.resize(_target_idxs.size(),1);

        Eigen::MatrixXd samples;
        samples.resize(p,nIterations);

        Eigen::VectorXd b;
        b.resize(n,1);

        for (int i = 0; i < _target_idxs.size(); i++) {
            ret[i] = 0;
        }

        for (_n_iterations = 0; _n_iterations < nIterations; _n_iterations++) {

            for (int i = 0; i < n; i++) {
                b[i] = _normal_dist(_re); //uniform, i.i.d. normal variables
            }

            Eigen::VectorXd rhs = (b.transpose()*_jacobian).transpose();

            if constexpr (SolverTraits<SolverT>::requireHessian) {
                samples.col(_n_iterations) = _solver.solve(rhs);
            } else {
                samples.col(_n_iterations) = _solver.solve(rhs);
            }

            auto status = _solver.info();
            if (status != Eigen::Success) {
                break;
            }
        }

        for (int i = 0; i < _target_idxs.size(); i++) {
            Idx const& idx = _target_idxs[i];
            if constexpr (Robust) {

                int nCols = samples.cols();
                std::vector<double> values (nCols);
                for (int i = 0; i < nCols; i++) {
                    values[i] = samples(idx.i,i)*samples(idx.j,i);
                }
                std::sort(values.begin(), values.end());

                int nIdx = nCols/2;
                int sIdx = nIdx/2;

                nIdx = nCols - 2*sIdx;

                if (nCols < 8) { //for small sample size take all samples
                    nIdx = nCols;
                    sIdx = 0;
                }

                double accumulated = 0;

                for (int i = sIdx; i < sIdx+nIdx; i++) {
                    accumulated += values[i];
                }

                accumulated /= nIdx;

                ret[i] = accumulated;

            } else {
                ret[i] = samples.row(idx.i).dot(samples.row(idx.j)) / nIterations;
            }
        }

        return ret;
    }

protected:

    RandomEngineT _re;

    JacMatT const& _jacobian;
    HMatT _Hessian;
    SolverT _solver;

    int _n_obs;
    int _n_params;

    int _n_iterations;
    std::vector<Idx> _target_idxs;
    std::normal_distribution<double> _normal_dist;

};

template<typename JacMatT, typename SolverT, typename HMatT = Eigen::SparseMatrix<double>,  typename RandomEngineT = std::default_random_engine>
using StochasticCovarianceFromJacobianRobustBootstrapEstimator = StochasticCovarianceFromJacobianBootstrapEstimator<JacMatT,SolverT, HMatT,  RandomEngineT, true>;

/*!
 * \brief The StochasticCovarianceEstimator class is an helper class to estimate the covariance matrix of large scale sparse problem via a stochastic algorithm
 *
 * The method is derived from the Hutchinson estimator, extended for estimation of the diagonal entries as described in https://arxiv.org/abs/2201.10684
 *
 * The estimator takes an hessian, and a set of indexes where the (co)variance should be computed
 */
template<typename SolverT, typename RandomEngineT = std::default_random_engine>
class StochasticCovarianceFromHessianHutchinsonEstimator {

public:

    struct Idx {
        int i;
        int j;
        inline bool operator<(Idx const& other) const {
            if (i == other.i) {
                return j < other.j;
            }
            return i < other.i;
        }
        inline bool operator==(Idx const& other) const {
            return i == other.i and  j == other.j;
        }
    };

    /*!
     * \brief StochasticCovarianceFromHessianEstimator build the  estimator from a solver
     * \param solver
     * \param vecSize
     * \param covariance_idxs
     */
    StochasticCovarianceFromHessianHutchinsonEstimator(SolverT & solver, int vecSize, std::vector<Idx> const& covariance_idxs) :
        _solver(solver),
        _vecSize(vecSize)
    {

        _n_iterations = 0;

        int n = _vecSize;
        std::set<Idx> inputIdxs;
        for (Idx const& idx : covariance_idxs) {
            if (idx.i < 0 or idx.i >= n) {
                continue;
            }
            if (idx.j < 0 or idx.j >= n) {
                continue;
            }

            if (idx.j < idx.i) {
                inputIdxs.insert(Idx{idx.j,idx.i});
            } else {
                inputIdxs.insert(idx);
            }
        }
        _target_idxs.resize(inputIdxs.size());
        std::copy(inputIdxs.begin(), inputIdxs.end(), _target_idxs.begin());
        std::sort(_target_idxs.begin(), _target_idxs.end());

        std::random_device rd;
        _re.seed(rd());
    }

    void seed(int seed) {
        _re.seed(seed);
    }

    /*!
     * \brief statusOk check if the status is ok
     * \return true if status of solver is still Eigen::Success
     */
    inline bool statusOk() const {
        return _solver.info() == Eigen::Success;
    }

    /*!
     * \brief nIterations quick access to previous number of iterations
     * \return return the last number of iterations that was used for computation
     */
    inline int nIterations() const {
        return _n_iterations;
    }

    std::vector<Idx> const& targetIdxs() const {
        return _target_idxs;
    }

    Eigen::VectorXd computeEstimates(int nIterations) {

        int n = _vecSize;

        Eigen::VectorXd ret;
        ret.resize(_target_idxs.size(),1);

        Eigen::VectorXd b;
        b.resize(n,1);

        for (int i = 0; i < _target_idxs.size(); i++) {
            ret[i] = 0;
        }

        std::uniform_int_distribution random_dist(0,1);

        int c = 0;
        for (_n_iterations = 0; _n_iterations < nIterations; _n_iterations++) {

            for (int i = 0; i < n; i++) {
                b[i] = random_dist(_re) == 1 ? 1 : -1; //randomly assign +1 or -1 (Rademacher variables)
            }

            Eigen::VectorXd x = _solver.solve(b);

            auto status = _solver.info();
            if (status != Eigen::Success) {
                break;
            }

            for (int i = 0; i < _target_idxs.size(); i++) {
                Idx const& idx = _target_idxs[i];
                ret[i] += b[idx.j]*x[idx.i];
            }
            c++;
        }

        ret /= c;

        return ret;
    }

protected:

    RandomEngineT _re;

    SolverT & _solver;
    int _vecSize;

    int _n_iterations;
    std::vector<Idx> _target_idxs;

};

/*!
 * \brief The StochasticCovarianceEstimator class is an helper class to estimate the covariance matrix of large scale sparse problem via a stochastic algorithm
 *
 * The method is derived from the Hutchinson estimator, extended for estimation of the diagonal entries as described in https://arxiv.org/abs/2201.10684
 *
 * The estimator takes an hessian, and a set of indexes where the (co)variance should be computed
 */
template<typename SolverT, typename RandomEngineT = std::default_random_engine>
class StochasticCovarianceFromHessianGaussianHutchinsonEstimator {

public:

    struct Idx {
        int i;
        int j;
        inline bool operator<(Idx const& other) const {
            if (i == other.i) {
                return j < other.j;
            }
            return i < other.i;
        }
        inline bool operator==(Idx const& other) const {
            return i == other.i and  j == other.j;
        }
    };

    /*!
     * \brief StochasticCovarianceFromHessianEstimator build the  estimator from a solver
     * \param solver
     * \param vecSize
     * \param covariance_idxs
     */
    StochasticCovarianceFromHessianGaussianHutchinsonEstimator(SolverT & solver, int vecSize, std::vector<Idx> const& covariance_idxs) :
        _solver(solver),
        _vecSize(vecSize)
    {

        _n_iterations = 0;

        int n = _vecSize;
        std::set<Idx> inputIdxs;
        for (Idx const& idx : covariance_idxs) {
            if (idx.i < 0 or idx.i >= n) {
                continue;
            }
            if (idx.j < 0 or idx.j >= n) {
                continue;
            }

            if (idx.j < idx.i) {
                inputIdxs.insert(Idx{idx.j,idx.i});
            } else {
                inputIdxs.insert(idx);
            }
        }
        _target_idxs.resize(inputIdxs.size());
        std::copy(inputIdxs.begin(), inputIdxs.end(), _target_idxs.begin());
        std::sort(_target_idxs.begin(), _target_idxs.end());

        std::random_device rd;
        _re.seed(rd());
    }

    void seed(int seed) {
        _re.seed(seed);
    }

    /*!
     * \brief statusOk check if the status is ok
     * \return true if status of solver is still Eigen::Success
     */
    inline bool statusOk() const {
        return _solver.info() == Eigen::Success;
    }

    /*!
     * \brief nIterations quick access to previous number of iterations
     * \return return the last number of iterations that was used for computation
     */
    inline int nIterations() const {
        return _n_iterations;
    }

    std::vector<Idx> const& targetIdxs() const {
        return _target_idxs;
    }

    Eigen::VectorXd computeEstimates(int nIterations) {

        int n = _vecSize;

        Eigen::VectorXd ret;
        ret.resize(_target_idxs.size(),1);

        Eigen::VectorXd b;
        b.resize(n,1);

        for (int i = 0; i < _target_idxs.size(); i++) {
            ret[i] = 0;
        }

        std::normal_distribution<double> random_dist(0,1);

        Eigen::VectorXd c;
        c.resize(n,1);

        for (_n_iterations = 0; _n_iterations < nIterations; _n_iterations++) {

            for (int i = 0; i < n; i++) {
                b[i] = random_dist(_re); //assign a sample from the normal distribution
            }

            Eigen::VectorXd x = _solver.solve(b);

            auto status = _solver.info();
            if (status != Eigen::Success) {
                break;
            }

            for (int i = 0; i < _target_idxs.size(); i++) {
                Idx const& idx = _target_idxs[i];
                ret[i] += b[idx.j]*x[idx.i];
            }

            for (int i = 0; i < n; i++) {
                c[i] += b[i]*b[i];
            }
        }

        ret.array() /= c.array();

        return ret;
    }

protected:

    RandomEngineT _re;

    SolverT & _solver;
    int _vecSize;

    int _n_iterations;
    std::vector<Idx> _target_idxs;

};

} // namespace StereoVisionApp

#endif // STOCHASTICSPARSECOVARIANCEESTIMATOR_H
