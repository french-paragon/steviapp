#include <QtTest/QtTest>

#include <random>
#include <time.h>

#include "datablocks/project.h"
#include "datablocks/image.h"
#include "datablocks/camera.h"
#include "datablocks/landmark.h"
#include "datablocks/correspondencesset.h"
#include "datablocks/localcoordinatesystem.h"
#include "datablocks/trajectory.h"
#include "datablocks/mounting.h"

#include "sparsesolver/stochasticsparsecovarianceestimator.h"
#include "sparsesolver/modularsbasolver.h"
#include "sparsesolver/sbamodules/landmarkssbamodule.h"
#include "sparsesolver/sbamodules/imagealignementsbamodule.h"
#include "sparsesolver/sbamodules/correspondencessetsbamodule.h"
#include "sparsesolver/sbamodules/trajectorybasesbamodule.h"

#include "testutils/standardprojectgenerators.h"
#include "testutils/datablocks/generatedtrajectory.h"

#include <glog/logging.h>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/QR>

#include <QDebug>
#include <QMetaObject>

template<typename T>
class DiagPlusLowRankBuilder {
public:

    using VecT = Eigen::Matrix<T, Eigen::Dynamic, 1>;
    using MatT = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

    DiagPlusLowRankBuilder(VecT const& diag, MatT const& lowRank) :
        _diag(diag),
        _lowRank(lowRank)
    {

    }

    VecT solve(VecT const& b) {
        return _diag.cwiseProduct(b) + _lowRank.transpose()*(_lowRank*b);
    }

    int info() {
        return Eigen::Success;
    }

    T getGroundTruth(int i, int j) {
        if (i == j) {
            return _diag[i] + _lowRank.col(i).dot(_lowRank.col(j));
        }
        return _lowRank.col(i).dot(_lowRank.col(j));
    }

protected:
    VecT const& _diag;
    MatT const& _lowRank;
};

template<typename T>
class TridiagonalBuilder {
public:

    using VecT = Eigen::Matrix<T, Eigen::Dynamic, 1>;
    using MatT = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

    TridiagonalBuilder(VecT const& diag, VecT const& secondDiag) :
        _diag(diag),
        _secondDiag(secondDiag)
    {

    }

    VecT solve(VecT const& b) {
        int n = b.rows();
        VecT ret = _diag.cwiseProduct(b);

        for (int i = 0; i < n-1; i++) {
            ret[i] += _secondDiag[i]*b[i+1];
        }
        for (int i = 1; i < n; i++) {
            ret[i] += _secondDiag[i-1]*b[i];
        }

        return ret;
    }

    int info() {
        return Eigen::Success;
    }

    T getGroundTruth(int i, int j) {
        if (i == j) {
            return _diag[i];
        }
        if (i == j-1) {
            return _secondDiag[i];
        }
        if (i == j+1) {
            return _secondDiag[j];
        }
        return 0;
    }

protected:
    VecT const& _diag;
    VecT const& _secondDiag;
};

class TestSparseCovEstimator : public QObject
{
    Q_OBJECT
public:

private Q_SLOTS:

    void initTestCase();

    void tridiagonal_data();
    void tridiagonal();

    void diagPlusLowRank_data();
    void diagPlusLowRank();

    void simplePnPHessian_data();
    void simplePnPHessian();

    void singleTrajectory_data();
    void singleTrajectory();

private :

    template<typename SolverT>
    struct SolverTraits {
        static constexpr bool IsIterative = false;
        static constexpr bool IsLeastSquare = false;
    };

    template<typename MatrixType_, int UpLo_, typename Preconditioner_>
    struct SolverTraits<Eigen::ConjugateGradient< MatrixType_, UpLo_, Preconditioner_ >> {
        static constexpr bool IsIterative = true;
        static constexpr bool IsLeastSquare = false;
    };

    template<typename MatrixType_, typename Preconditioner_>
        struct SolverTraits<Eigen::LeastSquaresConjugateGradient<MatrixType_, Preconditioner_ >> {
        static constexpr bool IsIterative = true;
        static constexpr bool IsLeastSquare = true;
    };

    enum EstimatorType {
        Bootstrap = 0,
        Hutchinson = 1
    };

    template <EstimatorType E, typename SparseHType, typename SolverT>
    void simplePnPHessianImpl(int nImages, int nSamples, bool offDiagonal) {

        StereoVisionApp::ProjectFactory& pF = StereoVisionApp::ProjectFactory::defaultProjectFactory();

        std::unique_ptr<StereoVisionApp::Project> pPtr(pF.createProject(this)); //use unique ptr to ensure project is deleted at the end of test case

        QVERIFY(pPtr != nullptr);

        StereoVisionApp::Project& project = *pPtr;

        constexpr int seed = 42;
        std::default_random_engine re(seed);

        bool projectGenerationOk = StereoVisionApp::StandardProjectGenerators::simplePnPGenerator(&project, seed, nImages);

        QVERIFY(projectGenerationOk);

        StereoVisionApp::ModularSBASolver sbaSolver(&project);
        sbaSolver.setSilent(true);

        bool lmModuleAdded = sbaSolver.addModule(new StereoVisionApp::LandmarksSBAModule());
        bool imModuleAdded = sbaSolver.addModule(new StereoVisionApp::ImageAlignementSBAModule());
        bool correspModuleAdded = sbaSolver.addModule(new StereoVisionApp::CorrespondencesSetSBAModule());

        QVERIFY(lmModuleAdded);
        QVERIFY(imModuleAdded);
        QVERIFY(correspModuleAdded);

        bool initSuccess = sbaSolver.init();

        ceres::Problem* problem = sbaSolver.ceresProblem();

        QVERIFY(initSuccess);

        QVERIFY(problem != nullptr);

        //check something was added in the factor graph
        QVERIFY(problem->NumResidualBlocks() > 0);

        double cost;

        ceres::Problem::EvaluateOptions options;

        std::vector<double*> all_parameter_blocks;
        problem->GetParameterBlocks(&all_parameter_blocks);
        std::vector<double*> variable_parameter_blocks;
        variable_parameter_blocks.reserve(all_parameter_blocks.size());

        for (double* paramBlock : all_parameter_blocks) {
            bool isFixed = problem->IsParameterBlockConstant(paramBlock);
            if (isFixed) {
                continue;
            }
            variable_parameter_blocks.push_back(paramBlock);
        }

        options.parameter_blocks = variable_parameter_blocks;

        ceres::CRSMatrix jacobian;
        bool evaluateOk = problem->Evaluate(options, &cost, nullptr, nullptr, &jacobian);

        QVERIFY(evaluateOk);

        QVERIFY(std::abs(cost) < 1e-6); //the problem should be initialized with a perfect solution

        Eigen::MappedSparseMatrix<double, Eigen::RowMajor> eigen_jacobian(
            jacobian.num_rows, jacobian.num_cols,
            jacobian.values.size(),
            jacobian.rows.data(), jacobian.cols.data(), jacobian.values.data());

        int n = eigen_jacobian.cols();

        qInfo() << "Hessian matrix has size " << n << "x" << n;

        SolverT solver;
        SparseHType H;

        if constexpr(SolverTraits<SolverT>::IsLeastSquare) {
            solver.compute(eigen_jacobian);
        } else {
            H = eigen_jacobian.transpose()*eigen_jacobian;
            solver.compute(H);
        }

        if constexpr(SolverTraits<SolverT>::IsIterative) {
            qInfo() << "running with iterative solver";
            solver.setMaxIterations(std::max(500,n));
            solver.setTolerance(1e-3);
        }

        using SparseCovarianceEstimate =
            typename std::conditional<E == Hutchinson,
                                        StereoVisionApp::StochasticCovarianceFromHessianHutchinsonEstimator<SolverT>,
                                      StereoVisionApp::StochasticCovarianceFromJacobianBootstrapEstimator<decltype(eigen_jacobian), SolverT, SparseHType>>::type;

        int nIdxs = (offDiagonal) ? n-1 : n;
        std::vector<typename SparseCovarianceEstimate::Idx> idxs(nIdxs);

        if (offDiagonal) {
            for (int i = 0; i < nIdxs; i++) {
                idxs[i] = {i,i+1};
            }
        } else {
            for (int i = 0; i < nIdxs; i++) {
                idxs[i] = {i,i};
            }
        }

        Eigen::VectorXd estimates;


        if constexpr (E == Bootstrap) {

            SparseCovarianceEstimate estimator(eigen_jacobian, idxs);
            estimator.seed(seed);
            std::vector<typename SparseCovarianceEstimate::Idx>  targets = estimator.targetIdxs();

            QCOMPARE(targets.size(), nIdxs);

            for (int i = 0; i < nIdxs; i++) {
                QCOMPARE(targets[i], idxs[i]);
            }

            QBENCHMARK_ONCE {
                estimates = estimator.computeEstimates(nSamples);
            }

            if constexpr(SolverTraits<SolverT>::IsIterative) {
                qInfo() << "# samples : " << estimator.nIterations();
                qInfo() << "# iterations : " << estimator.solverIterations();
                qInfo() << "estimated error : " << estimator.solverError();
            }

            bool ok = estimator.statusOk();

            if (!ok) {
                QSKIP("Failed to converge, test needs revision!");
            }

            QCOMPARE(estimator.nIterations(), nSamples);

        }

        if constexpr (E == Hutchinson) {

            SparseCovarianceEstimate estimator(solver, n, idxs);
            estimator.seed(seed);
            std::vector<typename SparseCovarianceEstimate::Idx>  targets = estimator.targetIdxs();

            QCOMPARE(targets.size(), nIdxs);

            for (int i = 0; i < nIdxs; i++) {
                QCOMPARE(targets[i], idxs[i]);
            }

            QBENCHMARK_ONCE {
                estimates = estimator.computeEstimates(nSamples);
            }

            if constexpr(SolverTraits<SolverT>::IsIterative) {
                qInfo() << "# samples : " << estimator.nIterations();
                qInfo() << "# iterations : " << solver.iterations();
                qInfo() << "estimated error : " << solver.error();
            }

            bool ok = estimator.statusOk();

            if (!ok) {
                QSKIP("Failed to converge, test needs revision!");
            }

            QCOMPARE(estimator.nIterations(), nSamples);

        }

        std::vector<int> diagIdxs(nIdxs);

        for (int i = 0; i < nIdxs; i++) {
            diagIdxs[i] = i;
        }

        constexpr int maxNTest = 1000;

        int nTested = std::min(nIdxs, maxNTest);

        if (nTested < nIdxs) {
            std::shuffle(diagIdxs.begin(), diagIdxs.end(), re);
        }

        int countInBound05 = 0;
        int countInBound10 = 0;
        int countInBound20 = 0;
        int countInBound50 = 0;
        int countInBound100 = 0;

        constexpr double minVarThresh = 1e-1;

        for (int i = 0; i < nTested; i++) {
            Eigen::VectorXd b = Eigen::VectorXd::Zero(n);
            auto idx = idxs[diagIdxs[i]];
            b[idx.j] = 1;
            Eigen::VectorXd x = solver.solve(b);
            double gt = x[idx.i];
            double est = estimates[diagIdxs[i]];
            double error = std::abs(est - gt);

            double thresh = std::max(std::abs(gt),minVarThresh);

            bool ok05 = error < 0.05*thresh;
            if (ok05) {
                countInBound05++;
            }

            bool ok10 = error < 0.1*thresh;
            if (ok10) {
                countInBound10++;
            }

            bool ok20 = error < 0.2*thresh;
            if (ok20) {
                countInBound20++;
            }

            bool ok50 = error < 0.5*thresh;
            if (ok50) {
                countInBound50++;
            }

            bool ok100 = error < thresh;
            if (ok100) {
                countInBound100++;
            }
        }

        qInfo() << countInBound05 << "/" << nTested << " coefficient are within 5% from gt";
        qInfo() << countInBound10 << "/" << nTested << " coefficient are within 10% of gt";
        qInfo() << countInBound20 << "/" << nTested << " coefficient are within 20% of gt";
        qInfo() << countInBound50 << "/" << nTested << " coefficient are within 50% of gt";
        qInfo() << countInBound100 << "/" << nTested << " coefficient are within 100% of gt";

    }

    template <EstimatorType E, typename SparseHType, typename SolverT>
    void singleTrajectoryImpl(int nSamples, float duration, float samplingDt, float accDt, float gpsDt, bool offDiagonal) {

        StereoVisionApp::ProjectFactory& pF = StereoVisionApp::ProjectFactory::defaultProjectFactory();

        StereoVisionApp::Project* pPtr = pF.createProject(this);

        QVERIFY(pPtr != nullptr);

        StereoVisionApp::Project& project = *pPtr;
        const char* ecefCRS = "EPSG:4978";

        project.setDefaultProjectCRS(ecefCRS);

        constexpr double EarthRadius = 6357000;

        StereoVision::Geometry::AffineTransform<double> ecef2Local(StereoVision::Geometry::rodriguezFormula(Eigen::Vector3d(0.27,-0.69,1.42)),
                                                                   Eigen::Vector3d(0,0,-EarthRadius-3342));

        project.setLocalCoordinateFrame(ecef2Local);

        constexpr int seed = 42;
        std::default_random_engine re(seed);

        bool ok = StereoVisionApp::StandardProjectGenerators::circularTrajectoryEcef(&project, seed, duration, samplingDt, accDt, gpsDt);

        QVERIFY(ok);

        QVector<qint64> traj_idxs = project.getIdsByClass(StereoVisionApp::Trajectory::staticMetaObject.className());
        QCOMPARE(traj_idxs.size(), 1);

        qint64 trajectoryId = traj_idxs[0];

        StereoVisionApp::Trajectory* traj = project.getDataBlock<StereoVisionApp::Trajectory>(trajectoryId);

        QVERIFY(traj != nullptr);

        StereoVisionApp::ModularSBASolver sbaSolver(&project);
        sbaSolver.setSilent(true);
        sbaSolver.setFuncTolerance(1e-8);
        sbaSolver.setParamsTolerance(1e-10);

        StereoVisionApp::TrajectoryBaseSBAModule* trajSBAModule =
            new StereoVisionApp::TrajectoryBaseSBAModule(traj->getPreIntegrationTime());
        bool trajModuleAdded = sbaSolver.addModule(trajSBAModule);

        QVERIFY(trajModuleAdded);

        bool initSuccess = sbaSolver.init();

        QVERIFY(initSuccess);

        QVERIFY(sbaSolver.itemIsObservable(trajectoryId));

        ceres::Problem* problem = sbaSolver.ceresProblem();

        QVERIFY(problem != nullptr);

        //check something was added in the factor graph
        QVERIFY(problem->NumResidualBlocks() > 0);        double cost;

        ceres::Problem::EvaluateOptions options;

        std::vector<double*> all_parameter_blocks;
        problem->GetParameterBlocks(&all_parameter_blocks);
        std::vector<double*> variable_parameter_blocks;
        variable_parameter_blocks.reserve(all_parameter_blocks.size());

        for (double* paramBlock : all_parameter_blocks) {
            bool isFixed = problem->IsParameterBlockConstant(paramBlock);
            if (isFixed) {
                continue;
            }
            variable_parameter_blocks.push_back(paramBlock);
        }

        options.parameter_blocks = variable_parameter_blocks;

        ceres::CRSMatrix jacobian;
        bool evaluateOk = problem->Evaluate(options, &cost, nullptr, nullptr, &jacobian);

        QVERIFY(evaluateOk);

        qInfo() << "Cost is: " << cost;

        Eigen::MappedSparseMatrix<double, Eigen::RowMajor> eigen_jacobian(
            jacobian.num_rows, jacobian.num_cols,
            jacobian.values.size(),
            jacobian.rows.data(), jacobian.cols.data(), jacobian.values.data());

        int n = eigen_jacobian.cols();

        qInfo() << "Hessian matrix has size " << n << "x" << n;

        SolverT solver;
        SparseHType H;

        if constexpr(SolverTraits<SolverT>::IsLeastSquare) {
            solver.compute(eigen_jacobian);
        } else {
            H = eigen_jacobian.transpose()*eigen_jacobian;
            solver.compute(H);
        }

        if constexpr(SolverTraits<SolverT>::IsIterative) {
            qInfo() << "running with iterative solver";
            solver.setMaxIterations(std::max(500,n));
            solver.setTolerance(1e-3);
        }

        using SparseCovarianceEstimate =
            typename std::conditional<E == Hutchinson,
                                      StereoVisionApp::StochasticCovarianceFromHessianHutchinsonEstimator<SolverT>,
                                      StereoVisionApp::StochasticCovarianceFromJacobianBootstrapEstimator<decltype(eigen_jacobian), SolverT, SparseHType>>::type;

        int nIdxs = (offDiagonal) ? n-1 : n;
        std::vector<typename SparseCovarianceEstimate::Idx> idxs(nIdxs);

        if (offDiagonal) {
            for (int i = 0; i < nIdxs; i++) {
                idxs[i] = {i,i+1};
            }
        } else {
            for (int i = 0; i < nIdxs; i++) {
                idxs[i] = {i,i};
            }
        }

        Eigen::VectorXd estimates;


        if constexpr (E == Bootstrap) {

            SparseCovarianceEstimate estimator(eigen_jacobian, idxs);
            estimator.seed(seed);
            std::vector<typename SparseCovarianceEstimate::Idx>  targets = estimator.targetIdxs();

            QCOMPARE(targets.size(), nIdxs);

            for (int i = 0; i < nIdxs; i++) {
                QCOMPARE(targets[i], idxs[i]);
            }

            QBENCHMARK_ONCE {
                estimates = estimator.computeEstimates(nSamples);
            }

            if constexpr(SolverTraits<SolverT>::IsIterative) {
                qInfo() << "# samples : " << estimator.nIterations();
                qInfo() << "# iterations : " << estimator.solverIterations();
                qInfo() << "estimated error : " << estimator.solverError();
            }

            bool ok = estimator.statusOk();

            if (!ok) {
                QSKIP("Failed to converge, test needs revision!");
            }

            QCOMPARE(estimator.nIterations(), nSamples);

        }

        if constexpr (E == Hutchinson) {

            SparseCovarianceEstimate estimator(solver, n, idxs);
            estimator.seed(seed);
            std::vector<typename SparseCovarianceEstimate::Idx>  targets = estimator.targetIdxs();

            QCOMPARE(targets.size(), nIdxs);

            for (int i = 0; i < nIdxs; i++) {
                QCOMPARE(targets[i], idxs[i]);
            }

            QBENCHMARK_ONCE {
                estimates = estimator.computeEstimates(nSamples);
            }

            if constexpr(SolverTraits<SolverT>::IsIterative) {
                qInfo() << "# samples : " << estimator.nIterations();
                qInfo() << "# iterations : " << solver.iterations();
                qInfo() << "estimated error : " << solver.error();
            }

            bool ok = estimator.statusOk();

            if (!ok) {
                QSKIP("Failed to converge, test needs revision!");
            }

            QCOMPARE(estimator.nIterations(), nSamples);

        }

        std::vector<int> diagIdxs(nIdxs);

        for (int i = 0; i < nIdxs; i++) {
            diagIdxs[i] = i;
        }

        constexpr int maxNTest = 1000;

        int nTested = std::min(nIdxs, maxNTest);

        if (nTested < nIdxs) {
            std::shuffle(diagIdxs.begin(), diagIdxs.end(), re);
        }

        int countInBound05 = 0;
        int countInBound10 = 0;
        int countInBound20 = 0;
        int countInBound50 = 0;
        int countInBound100 = 0;

        constexpr double minVarThresh = 1e-1;

        for (int i = 0; i < nTested; i++) {
            Eigen::VectorXd b = Eigen::VectorXd::Zero(n);
            auto idx = idxs[diagIdxs[i]];
            b[idx.j] = 1;
            Eigen::VectorXd x = solver.solve(b);
            double gt = x[idx.i];
            double est = estimates[diagIdxs[i]];
            double error = std::abs(est - gt);

            double thresh = std::max(std::abs(gt),minVarThresh);

            bool ok05 = error < 0.05*thresh;
            if (ok05) {
                countInBound05++;
            }

            bool ok10 = error < 0.1*thresh;
            if (ok10) {
                countInBound10++;
            }

            bool ok20 = error < 0.2*thresh;
            if (ok20) {
                countInBound20++;
            }

            bool ok50 = error < 0.5*thresh;
            if (ok50) {
                countInBound50++;
            }

            bool ok100 = error < thresh;
            if (ok100) {
                countInBound100++;
            }
        }

        qInfo() << countInBound05 << "/" << nTested << " coefficient are within 5% from gt";
        qInfo() << countInBound10 << "/" << nTested << " coefficient are within 10% of gt";
        qInfo() << countInBound20 << "/" << nTested << " coefficient are within 20% of gt";
        qInfo() << countInBound50 << "/" << nTested << " coefficient are within 50% of gt";
        qInfo() << countInBound100 << "/" << nTested << " coefficient are within 100% of gt";

    }

};


void TestSparseCovEstimator::initTestCase() {

    srand(time(nullptr));

    //configure libraries
    google::InitGoogleLogging("TestSparseCovarianceEstimator");

    StereoVisionApp::ProjectFactory& pF = StereoVisionApp::ProjectFactory::defaultProjectFactory();

    pF.addType(new StereoVisionApp::LandmarkFactory(this));
    pF.addType(new StereoVisionApp::ImageFactory(this));
    pF.addType(new StereoVisionApp::CameraFactory(this));
    pF.addType(new StereoVisionApp::CorrespondencesSetFactory(this));
    pF.addType(new StereoVisionApp::LocalCoordinateSystemFactory(this));
    pF.addType(new StereoVisionApp::GeneratedTrajectoryFactory(this)); //ensure the trajectores can be configured with generators
    pF.addType(new StereoVisionApp::MountingFactory(this));

}


void TestSparseCovEstimator::tridiagonal_data() {

    QTest::addColumn<bool>("offDiagonal");
    QTest::addColumn<int>("n");
    QTest::addColumn<int>("nSamples");
    QTest::addColumn<double>("mainScale");
    QTest::addColumn<double>("secondScale");

    QTest::newRow("size 5k x 5k 3 samples diagonal") << false << 5000 << 3 << 10. << 0.1;
    QTest::newRow("size 5k x 5k 10 samples diagonal") << false << 5000 << 10 << 10. << 0.1;
    QTest::newRow("size 5k x 5k 100 samples diagonal") << false << 5000 << 100 << 10. << 0.1;

    QTest::newRow("size 100k x 100k 10 samples diagonal") << false << 100000 << 10 << 10. << 0.1;
    QTest::newRow("size 100k x 100k 100 samples diagonal") << false << 100000 << 100 << 10. << 0.1;
    QTest::newRow("size 100k x 100k 1000 samples diagonal") << false << 100000 << 1000 << 10. << 0.1;

    QTest::newRow("size 5k x 5k 3 samples off-diagonal") << true << 5000 << 3 << 10. << 0.1;
    QTest::newRow("size 5k x 5k 10 samples off-diagonal") << true << 5000 << 10 << 10. << 0.1;
    QTest::newRow("size 5k x 5k 100 samples off-diagonal") << true << 5000 << 100 << 10. << 0.1;

    QTest::newRow("size 100k x 100k 10 samples off-diagonal") << true << 100000 << 10 << 10. << 0.1;
    QTest::newRow("size 100k x 100k 100 samples off-diagonal") << true << 100000 << 100 << 10. << 0.1;
    QTest::newRow("size 100k x 100k 1000 samples off-diagonal") << true << 100000 << 1000 << 10. << 0.1;

    QTest::newRow("size 5k x 5k 3 samples outter-diagonal") << true << 5000 << 3 << 0. << 10.;
    QTest::newRow("size 5k x 5k 10 samples outter-diagonal") << true << 5000 << 10 << 0. << 10.;
    QTest::newRow("size 5k x 5k 100 samples outter-diagonal") << true << 5000 << 100 << 0. << 10.;

    QTest::newRow("size 100k x 100k 10 samples outter-diagonal") << true << 100000 << 10 << 0. << 10.;
    QTest::newRow("size 100k x 100k 100 samples outter-diagonal") << true << 100000 << 100 << 0. << 10.;
    QTest::newRow("size 100k x 100k 1000 samples outter-diagonal") << true << 100000 << 1000 << 0. << 10.;

}
void TestSparseCovEstimator::tridiagonal() {

    QFETCH(bool, offDiagonal);
    QFETCH(int, n);
    QFETCH(int, nSamples);
    QFETCH(double, mainScale);
    QFETCH(double, secondScale);

    constexpr int seed = 42;

    std::default_random_engine re;
    re.seed(seed);

    std::uniform_real_distribution<double> mainDist(-mainScale, mainScale);
    std::uniform_real_distribution<double> secondDist(-secondScale, secondScale);

    Eigen::VectorXd mainDiag;
    mainDiag.resize(n);
    Eigen::VectorXd secondDiag;
    secondDiag.resize(n-1);

    for (int i = 0; i < n; i++) {
        mainDiag[i] = mainDist(re);
    }

    for (int i = 0; i < n-1; i++) {
        secondDiag[i] = secondDist(re);
    }

    using SolvT = TridiagonalBuilder<double>;

    TridiagonalBuilder pseudoSolver(mainDiag, secondDiag);

    using SparseCovarianceEstimate =
        StereoVisionApp::StochasticCovarianceFromHessianHutchinsonEstimator<SolvT>;

    std::vector<typename SparseCovarianceEstimate::Idx> idxs(n);

    if (offDiagonal) {
        for (int i = 0; i < n-1; i++) {
            idxs[i] = {i,i+1};
        }
    } else {
        for (int i = 0; i < n; i++) {
            idxs[i] = {i,i};
        }
    }

    SparseCovarianceEstimate estimator(pseudoSolver, n, idxs);
    estimator.seed(seed);
    auto targets = estimator.targetIdxs();

    Eigen::VectorXd estimates;

    estimates = estimator.computeEstimates(nSamples);

    bool ok = estimator.statusOk();

    if (!ok) {
        QSKIP("Failed to converge, test needs revision!");
    }

    QCOMPARE(estimator.nIterations(), nSamples);

    std::vector<int> diagIdxs(n);

    for (int i = 0; i < n; i++) {
        diagIdxs[i] = i;
    }

    constexpr int maxNTest = 1000;

    int nTested = std::min(n, maxNTest);

    if (nTested < n) {
        std::shuffle(diagIdxs.begin(), diagIdxs.end(), re);
    }

    int countInBound05 = 0;
    int countInBound10 = 0;
    int countInBound20 = 0;
    int countInBound50 = 0;
    int countInBound100 = 0;

    for (int i = 0; i < nTested; i++) {
        auto idx = idxs[diagIdxs[i]];
        double gt = pseudoSolver.getGroundTruth(idx.i, idx.j);
        double est = estimates[diagIdxs[i]];
        double error = std::abs(est - gt);

        double thresh = (offDiagonal) ? std::abs(secondScale) : std::abs(mainScale);

        bool ok05 = error < 0.05*thresh;
        if (ok05) {
            countInBound05++;
        }

        bool ok10 = error < 0.1*thresh;
        if (ok10) {
            countInBound10++;
        }

        bool ok20 = error < 0.2*thresh;
        if (ok20) {
            countInBound20++;
        }

        bool ok50 = error < 0.5*thresh;
        if (ok50) {
            countInBound50++;
        }

        bool ok100 = error < thresh;
        if (ok100) {
            countInBound100++;
        }
    }

    qInfo() << countInBound05 << "/" << nTested << " coefficient are within 5% of expected scale from gt";
    qInfo() << countInBound10 << "/" << nTested << " coefficient are within 10% of expected scale from gt";
    qInfo() << countInBound20 << "/" << nTested << " coefficient are within 20% of expected scale from gt";
    qInfo() << countInBound50 << "/" << nTested << " coefficient are within 50% of expected scale from gt";
    qInfo() << countInBound100 << "/" << nTested << " coefficient are within 100% of expected scale from gt";

}

void TestSparseCovEstimator::diagPlusLowRank_data() {

    QTest::addColumn<bool>("offDiagonal");
    QTest::addColumn<bool>("lowRankNormalized");
    QTest::addColumn<int>("n");
    QTest::addColumn<int>("r");
    QTest::addColumn<int>("nSamples");
    QTest::addColumn<double>("correlationRate");
    QTest::addColumn<double>("diagScale");
    QTest::addColumn<double>("lowRankScale");

    QTest::newRow("correlation l=0.5 5k x 5k 10 samples diagonal") << false << true << 5000 << 20 << 10 << 0.5 << 0. << 1.;
    QTest::newRow("correlation l=0.5 5k x 5k 100 samples diagonal") << false << true << 5000 << 20 << 100 << 0.5 << 0. << 1.;
    QTest::newRow("correlation l=0.5 5k x 5k 1000 samples diagonal") << false << true << 5000 << 20 << 1000 << 0.5 << 0. << 1.;

    QTest::newRow("correlation l=0.5 5k x 5k 10 samples off-diagonal") << true << true << 5000 << 20 << 10 << 0.5 << 0. << 1.;
    QTest::newRow("correlation l=0.5 5k x 5k 100 samples off-diagonal") << true << true << 5000 << 20 << 100 << 0.5 << 0. << 1.;
    QTest::newRow("correlation l=0.5 5k x 5k 1000 samples off-diagonal") << true << true << 5000 << 20 << 1000 << 0.5 << 0. << 1.;

    QTest::newRow("correlation l=0.5 100k x 100k 10 samples off-diagonal") << true << true << 100000 << 20 << 10 << 0.5 << 0. << 1.;
    QTest::newRow("correlation l=0.5 100k x 100k 100 samples off-diagonal") << true << true << 100000 << 20 << 100 << 0.5 << 0. << 1.;
    QTest::newRow("correlation l=0.5 100k x 100k 1000 samples off-diagonal") << true << true << 100000 << 20 << 1000 << 0.5 << 0. << 1.;

    QTest::newRow("correlation l=0.0 5k x 5k 10 samples diagonal") << false << true << 5000 << 20 << 10 << 0. << 0. << 1.;
    QTest::newRow("correlation l=0.0 5k x 5k 100 samples diagonal") << false << true << 5000 << 20 << 100 << 0. << 0. << 1.;
    QTest::newRow("correlation l=0.0 5k x 5k 1000 samples diagonal") << false << true << 5000 << 20 << 1000 << 0. << 0. << 1.;

    QTest::newRow("correlation l=0.0 5k x 5k 10 samples off-diagonal") << true << true << 5000 << 20 << 10 << 0. << 0. << 1.;
    QTest::newRow("correlation l=0.0 5k x 5k 100 samples off-diagonal") << true << true << 5000 << 20 << 100 << 0. << 0. << 1.;
    QTest::newRow("correlation l=0.0 5k x 5k 1000 samples off-diagonal") << true << true << 5000 << 20 << 1000 << 0. << 0. << 1.;

    QTest::newRow("correlation l=0.0 100k x 100k 10 samples off-diagonal") << true << true << 100000 << 20 << 10 << 0. << 0. << 1.;
    QTest::newRow("correlation l=0.0 100k x 100k 100 samples off-diagonal") << true << true << 100000 << 20 << 100 << 0. << 0. << 1.;
    QTest::newRow("correlation l=0.0 100k x 100k 1000 samples off-diagonal") << true << true << 100000 << 20 << 1000 << 0. << 0. << 1.;

    QTest::newRow("size 5k x 5k 3 samples diagonal") << false << false << 5000 << 20 << 3 << 0.5 << 10. << 0.1;
    QTest::newRow("size 5k x 5k 10 samples diagonal") << false << false << 5000 << 20 << 10 << 0.5 << 10. << 0.1;
    QTest::newRow("size 5k x 5k 100 samples diagonal") << false << false << 5000 << 20 << 100 << 0.5 << 10. << 0.1;

    QTest::newRow("size 100k x 100k 10 samples diagonal") << false << false << 100000 << 50 << 10 << 0.5 << 10. << 0.1;
    QTest::newRow("size 100k x 100k 100 samples diagonal") << false << false << 100000 << 50 << 100 << 0.5 << 10. << 0.1;
    QTest::newRow("size 100k x 100k 1000 samples diagonal") << false << false << 100000 << 50 << 1000 << 0.5 << 10. << 0.1;

    QTest::newRow("size 5k x 5k 3 samples off-diagonal") << true << false << 5000 << 20 << 3 << 0.5 << 10. << 0.1;
    QTest::newRow("size 5k x 5k 10 samples off-diagonal") << true << false << 5000 << 20 << 10 << 0.5 << 10. << 0.1;
    QTest::newRow("size 5k x 5k 100 samples off-diagonal") << true << false << 5000 << 20 << 100 << 0.5 << 10. << 0.1;

    QTest::newRow("size 100k x 100k 10 samples off-diagonal") << true << false << 100000 << 50 << 10 << 0.5 << 10. << 0.1;
    QTest::newRow("size 100k x 100k 100 samples off-diagonal") << true << false << 100000 << 50 << 100 << 0.5 << 10. << 0.1;
    QTest::newRow("size 100k x 100k 1000 samples off-diagonal") << true << false << 100000 << 50 << 1000 << 0.5 << 10. << 0.1;

}
void TestSparseCovEstimator::diagPlusLowRank() {

    QFETCH(bool, offDiagonal);
    QFETCH(bool, lowRankNormalized);
    QFETCH(int, n);
    QFETCH(int, r);
    QFETCH(int, nSamples);
    QFETCH(double, correlationRate);
    QFETCH(double, diagScale);
    QFETCH(double, lowRankScale);

    constexpr int seed = 42;

    bool rescaleLowRank = lowRankScale;
    if (lowRankNormalized) { //
        lowRankScale = 1;
    }

    std::default_random_engine re;
    re.seed(seed);

    std::uniform_real_distribution<double> diagDist(-diagScale, diagScale);
    std::uniform_real_distribution<double> lowRankDist(-lowRankScale, lowRankScale);
    std::uniform_real_distribution<double> correlationDist(0, correlationRate);

    Eigen::VectorXd diag;
    diag.resize(n);
    Eigen::MatrixXd lowRank;
    lowRank.resize(r,n);

    for (int i = 0; i < n; i++) {
        diag[i] = diagDist(re);
        for (int j = 0; j < r; j++) {
            double lambda = correlationDist(re);
            lowRank(j,i) = lowRankDist(re);
            if (i > 0) {
                lowRank(j,i) = lambda*lowRank(j,i-1)+(1-lambda)*lowRank(j,i);
            }
        }

        if (lowRankNormalized) { //
            double norm = lowRank.col(i).norm();
            lowRank.col(i) *= rescaleLowRank/norm;
        }
    }

    using SolvT = DiagPlusLowRankBuilder<double>;

    SolvT pseudoSolver(diag, lowRank);

    using SparseCovarianceEstimate =
        StereoVisionApp::StochasticCovarianceFromHessianHutchinsonEstimator<SolvT>;

    std::vector<typename SparseCovarianceEstimate::Idx> idxs(n);

    if (offDiagonal) {
        for (int i = 0; i < n-1; i++) {
            idxs[i] = {i,i+1};
        }
    } else {
        for (int i = 0; i < n; i++) {
            idxs[i] = {i,i};
        }
    }

    SparseCovarianceEstimate estimator(pseudoSolver, n, idxs);
    estimator.seed(seed);
    auto targets = estimator.targetIdxs();

    Eigen::VectorXd estimates;

    estimates = estimator.computeEstimates(nSamples);

    bool ok = estimator.statusOk();

    if (!ok) {
        QSKIP("Failed to converge, test needs revision!");
    }

    QCOMPARE(estimator.nIterations(), nSamples);

    std::vector<int> diagIdxs(n);

    for (int i = 0; i < n; i++) {
        diagIdxs[i] = i;
    }

    constexpr int maxNTest = 1000;

    int nTested = std::min(n, maxNTest);

    if (nTested < n) {
        std::shuffle(diagIdxs.begin(), diagIdxs.end(), re);
    }

    int countInBound05 = 0;
    int countInBound10 = 0;
    int countInBound20 = 0;
    int countInBound50 = 0;
    int countInBound100 = 0;

    for (int i = 0; i < nTested; i++) {
        auto idx = idxs[diagIdxs[i]];
        double gt = pseudoSolver.getGroundTruth(idx.i, idx.j);
        double est = estimates[diagIdxs[i]];
        double error = std::abs(est - gt);

        double thresh = (offDiagonal) ? std::abs(lowRankScale) : std::max(std::abs(lowRankScale),std::abs(diagScale));

        bool ok05 = error < 0.05*thresh;
        if (ok05) {
            countInBound05++;
        }

        bool ok10 = error < 0.1*thresh;
        if (ok10) {
            countInBound10++;
        }

        bool ok20 = error < 0.2*thresh;
        if (ok20) {
            countInBound20++;
        }

        bool ok50 = error < 0.5*thresh;
        if (ok50) {
            countInBound50++;
        }

        bool ok100 = error < thresh;
        if (ok100) {
            countInBound100++;
        }
    }

    qInfo() << countInBound05 << "/" << nTested << " coefficient are within 5% of expected scale from gt";
    qInfo() << countInBound10 << "/" << nTested << " coefficient are within 10% of expected scale from gt";
    qInfo() << countInBound20 << "/" << nTested << " coefficient are within 20% of expected scale from gt";
    qInfo() << countInBound50 << "/" << nTested << " coefficient are within 50% of expected scale from gt";
    qInfo() << countInBound100 << "/" << nTested << " coefficient are within 100% of expected scale from gt";
}

void TestSparseCovEstimator::simplePnPHessian_data() {

    QTest::addColumn<bool>("offDiagonal");
    QTest::addColumn<int>("estimatorType");
    QTest::addColumn<int>("nImages");
    QTest::addColumn<int>("nSamples");

    QTest::newRow("size200 Bootstrap 10 samples diagonal") << false << static_cast<int>(Bootstrap) << 200 << 10;
    QTest::newRow("size200 Bootstrap 100 samples diagonal") << false << static_cast<int>(Bootstrap) << 200 << 100;
    QTest::newRow("size200 Bootstrap 300 samples diagonal") << false << static_cast<int>(Bootstrap) << 200 << 300;

    QTest::newRow("size1000 Bootstrap 10 samples diagonal") << false << static_cast<int>(Bootstrap) << 1000 << 10;
    QTest::newRow("size1000 Bootstrap 100 samples diagonal") << false << static_cast<int>(Bootstrap) << 1000 << 100;
    QTest::newRow("size1000 Bootstrap 300 samples diagonal") << false << static_cast<int>(Bootstrap) << 1000 << 300;

    QTest::newRow("size200 Hutchinson 10 samples diagonal") << false << static_cast<int>(Hutchinson) << 200 << 10;
    QTest::newRow("size200 Hutchinson 100 samples diagonal") << false << static_cast<int>(Hutchinson) << 200 << 100;
    QTest::newRow("size200 Hutchinson 300 samples diagonal") << false << static_cast<int>(Hutchinson) << 200 << 300;

    QTest::newRow("size1000 Hutchinson 10 samples diagonal") << false << static_cast<int>(Hutchinson) << 1000 << 10;
    QTest::newRow("size1000 Hutchinson 100 samples diagonal") << false << static_cast<int>(Hutchinson) << 1000 << 100;
    QTest::newRow("size1000 Hutchinson 300 samples diagonal") << false << static_cast<int>(Hutchinson) << 1000 << 300;

    QTest::newRow("size200 Bootstrap 10 samples off-diagonal") << true << static_cast<int>(Bootstrap) << 200 << 10;
    QTest::newRow("size200 Bootstrap 100 samples off-diagonal") << true << static_cast<int>(Bootstrap) << 200 << 100;
    QTest::newRow("size200 Bootstrap 300 samples off-diagonal") << true << static_cast<int>(Bootstrap) << 200 << 300;
    QTest::newRow("size200 Bootstrap 1000 samples off-diagonal") << true << static_cast<int>(Bootstrap) << 200 << 1000;
    QTest::newRow("size200 Bootstrap 5000 samples off-diagonal") << true << static_cast<int>(Bootstrap) << 200 << 5000;

    QTest::newRow("size1000 Bootstrap 10 samples off-diagonal") << true << static_cast<int>(Bootstrap) << 1000 << 10;
    QTest::newRow("size1000 Bootstrap 100 samples off-diagonal") << true << static_cast<int>(Bootstrap) << 1000 << 100;
    QTest::newRow("size1000 Bootstrap 300 samples off-diagonal") << true << static_cast<int>(Bootstrap) << 1000 << 300;

    QTest::newRow("size200 Hutchinson 10 samples off-diagonal") << true << static_cast<int>(Hutchinson) << 200 << 10;
    QTest::newRow("size200 Hutchinson 100 samples off-diagonal") << true << static_cast<int>(Hutchinson) << 200 << 100;
    QTest::newRow("size200 Hutchinson 300 samples off-diagonal") << true << static_cast<int>(Hutchinson) << 200 << 300;
    QTest::newRow("size200 Hutchinson 1000 samples off-diagonal") << true << static_cast<int>(Hutchinson) << 200 << 1000;
    QTest::newRow("size200 Hutchinson 5000 samples off-diagonal") << true << static_cast<int>(Hutchinson) << 200 << 5000;

    QTest::newRow("size1000 Hutchinson 10 samples off-diagonal") << true << static_cast<int>(Hutchinson) << 1000 << 10;
    QTest::newRow("size1000 Hutchinson 100 samples off-diagonal") << true << static_cast<int>(Hutchinson) << 1000 << 100;
    QTest::newRow("size1000 Hutchinson 300 samples off-diagonal") << true << static_cast<int>(Hutchinson) << 1000 << 300;


}
void TestSparseCovEstimator::simplePnPHessian() {

    QFETCH(bool, offDiagonal);
    QFETCH(int, estimatorType);
    QFETCH(int, nImages);
    QFETCH(int, nSamples);

    using SparseHType = Eigen::SparseMatrix<double>;

    switch (estimatorType) {
    case Bootstrap:
        if (nImages < 300) {
            simplePnPHessianImpl<Bootstrap, SparseHType, Eigen::SparseQR<SparseHType, Eigen::COLAMDOrdering<int>>>(nImages, nSamples, offDiagonal);
        } else { //, Eigen::IncompleteCholesky<SparseHType>
            simplePnPHessianImpl<Bootstrap, SparseHType, Eigen::ConjugateGradient<SparseHType, Eigen::Upper|Eigen::Lower>>(nImages, nSamples, offDiagonal);
        }
        break;
    case Hutchinson:
        if (nImages < 300) {
            simplePnPHessianImpl<Hutchinson, SparseHType, Eigen::SparseQR<SparseHType, Eigen::COLAMDOrdering<int>>>(nImages, nSamples, offDiagonal);
        } else { //, Eigen::IncompleteCholesky<SparseHType>
            simplePnPHessianImpl<Hutchinson, SparseHType, Eigen::ConjugateGradient<SparseHType, Eigen::Upper|Eigen::Lower>>(nImages, nSamples, offDiagonal);
        }
        break;
    }


}

void TestSparseCovEstimator::singleTrajectory_data() {

    QTest::addColumn<bool>("offDiagonal");
    QTest::addColumn<int>("estimatorType");
    QTest::addColumn<int>("nSamples");
    QTest::addColumn<double>("duration");
    QTest::addColumn<double>("samplingDt");
    QTest::addColumn<double>("accDt");
    QTest::addColumn<double>("gpsDt");

    QTest::newRow("200 nodes Bootstrap 10 samples diagonal") << false << static_cast<int>(Bootstrap) << 10 << 100. << 0.5 << 0.1 << 5.;
    QTest::newRow("200 nodes Bootstrap 100 samples diagonal") << false << static_cast<int>(Bootstrap) << 100 << 100. << 0.5 << 0.1 << 5.;
    QTest::newRow("200 nodes Bootstrap 300 samples diagonal") << false << static_cast<int>(Bootstrap) << 300 << 100. << 0.5 << 0.1 << 5.;

    QTest::newRow("200 nodes Bootstrap 10 samples off-diagonal") << true << static_cast<int>(Bootstrap) << 10 << 100. << 0.5 << 0.1 << 5.;
    QTest::newRow("200 nodes Bootstrap 100 samples off-diagonal") << true << static_cast<int>(Bootstrap) << 100 << 100. << 0.5 << 0.1 << 5.;
    QTest::newRow("200 nodes Bootstrap 300 samples off-diagonal") << true << static_cast<int>(Bootstrap) << 300 << 100. << 0.5 << 0.1 << 5.;

    QTest::newRow("200 nodes Hutchinson 10 samples diagonal") << false << static_cast<int>(Hutchinson) << 10 << 100. << 0.5 << 0.1 << 5.;
    QTest::newRow("200 nodes Hutchinson 100 samples diagonal") << false << static_cast<int>(Hutchinson) << 100 << 100. << 0.5 << 0.1 << 5.;
    QTest::newRow("200 nodes Hutchinson 300 samples diagonal") << false << static_cast<int>(Hutchinson) << 300 << 100. << 0.5 << 0.1 << 5.;

    QTest::newRow("200 nodes Hutchinson 10 samples off-diagonal") << true << static_cast<int>(Hutchinson) << 10 << 100. << 0.5 << 0.1 << 5.;
    QTest::newRow("200 nodes Hutchinson 100 samples off-diagonal") << true << static_cast<int>(Hutchinson) << 100 << 100. << 0.5 << 0.1 << 5.;
    QTest::newRow("200 nodes Hutchinson 300 samples off-diagonal") << true << static_cast<int>(Hutchinson) << 300 << 100. << 0.5 << 0.1 << 5.;

}
void TestSparseCovEstimator::singleTrajectory() {

    QFETCH(bool, offDiagonal);
    QFETCH(int, estimatorType);
    QFETCH(int, nSamples);
    QFETCH(double, duration);
    QFETCH(double, samplingDt);
    QFETCH(double, accDt);
    QFETCH(double, gpsDt);

    using SparseHType = Eigen::SparseMatrix<double>;

    int nNodesEst = duration/samplingDt;

    switch (estimatorType) {
    case Bootstrap:
        if (nNodesEst < 200) {
            singleTrajectoryImpl<Bootstrap, SparseHType, Eigen::SparseQR<SparseHType, Eigen::COLAMDOrdering<int>>>
                (nSamples, duration, samplingDt, accDt, gpsDt, offDiagonal);
        } else { //, Eigen::IncompleteCholesky<SparseHType>
            singleTrajectoryImpl<Bootstrap, SparseHType, Eigen::ConjugateGradient<SparseHType, Eigen::Upper|Eigen::Lower>>
                (nSamples, duration, samplingDt, accDt, gpsDt, offDiagonal);
        }
        break;
    case Hutchinson:
        if (nNodesEst < 200) {
            singleTrajectoryImpl<Hutchinson, SparseHType, Eigen::SparseQR<SparseHType, Eigen::COLAMDOrdering<int>>>
                (nSamples, duration, samplingDt, accDt, gpsDt, offDiagonal);
        } else { //, Eigen::IncompleteCholesky<SparseHType>
            singleTrajectoryImpl<Hutchinson, SparseHType, Eigen::ConjugateGradient<SparseHType, Eigen::Upper|Eigen::Lower>>
                (nSamples, duration, samplingDt, accDt, gpsDt, offDiagonal);
        }
        break;
    }
}

QTEST_MAIN(TestSparseCovEstimator);
#include "main.moc"
