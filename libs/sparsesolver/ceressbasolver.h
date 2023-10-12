#ifndef STEREOVISIONAPP_CERESSBASOLVER_H
#define STEREOVISIONAPP_CERESSBASOLVER_H

#include "./sparsesolverbase.h"

#include <ceres/ceres.h>

#include <QMap>

namespace StereoVisionApp {

class Image;

class CeresSBASolver : public SparseSolverBase
{
public:
    CeresSBASolver(Project* p, bool computeUncertainty = true, bool sparse = true, QObject* parent = nullptr);

    int uncertaintySteps() const override;
    bool hasUncertaintyStep() const override;

protected:

    bool init() override;
    bool opt_step() override;
    bool std_step() override;
    bool writeResults() override;
    bool writeUncertainty() override;
    void cleanup() override;

    bool splitOptSteps() const override;

    qint64 setupCameraVertexForImage(Image* img);

    bool _sparse;
    bool _compute_marginals;
    bool _not_first_step;

    ceres::Problem _problem;

    struct LandmarkPos {
        std::array<double, 3> position;
    };

    std::vector<LandmarkPos> _landmarksParameters;
    QMap<qint64, std::size_t> _landmarkParametersIndex;

    struct CameraIntrinsicParameters {
        double fLen; //f
        std::array<double, 2> principalPoint; //pp
        std::array<double, 3> radialDistortion; //k1, k2, k3
        std::array<double, 2> tangentialDistortion; //t1, t2
        std::array<double, 2> skewDistortion; //B1, B2
    };

    std::vector<CameraIntrinsicParameters> _camParameters;
    QMap<qint64, std::size_t> _camParametersIndex;

    struct FramePoseParameters {
        std::array<double, 3> rAxis; //we go with the rotation axis representation. It is compressed, can be converted and allow to set priors more easily
        std::array<double, 3> t;
    };

    std::vector<FramePoseParameters> _frameParameters;
    QMap<qint64, std::size_t> _frameParametersIndex;

    std::vector<FramePoseParameters> _localCoordinatesParameters;
    QMap<qint64, std::size_t> _localCoordinatesParametersIndex;

    friend class CeresSBASolverIterationCallback;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_CERESSBASOLVER_H
