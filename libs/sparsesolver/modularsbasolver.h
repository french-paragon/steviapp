#ifndef STEREOVISIONAPP_MODULARSBASOLVER_H
#define STEREOVISIONAPP_MODULARSBASOLVER_H

#include "./sparsesolverbase.h"

#include <ceres/ceres.h>

#include <QVector>
#include <QMap>

#include <Eigen/Core>
#include <StereoVision/geometry/rotations.h>

class QTextStream;

namespace StereoVisionApp {

class Camera;

/*!
 * \brief The ModularSBASolver class is a modular solver for bundle adjustement problems.
 *
 * It uses abstract block for the factor graph, with blocks prebuilt for trajectories, frame observations, ...
 * It also provide an index of the landmarks, trajectories and camera that were added to the factor graph
 */
class ModularSBASolver: public SparseSolverBase
{
public:

    /*!
     * \brief The SBAModule class represent a functionality of the ModularSBASolver
     *
     * The module contains a familly of virtual functions that can be called by the ModularSBASolver
     * to extend the factor graph or collect outputs.
     */
    class SBAModule {
    public:

        SBAModule();
        virtual ~SBAModule();

        virtual bool init(ModularSBASolver* solver, ceres::Problem & problem) = 0;
        virtual bool writeResults(ModularSBASolver* solver) = 0;
        virtual bool writeUncertainty(ModularSBASolver* solver) = 0;
        virtual void cleanup(ModularSBASolver* solver) = 0;
    };

    /*!
     * \brief The ProjectorModule class can instanciace arbitrary projection cost function.
     *
     * This is used to encode different camera models easily.
     */
    class ProjectorModule {
    public:

        ProjectorModule();
        virtual ~ProjectorModule();

        /*!
         * \brief addProjectionCostFunction add a projection between a point and a pose
         * \param pointData the pointer to the optimizable point position data.
         * \param poseOrientation the pointer to the optimizable pose orientation (rotation axis).
         * \param posePosition the pointer to the optimizable pose position.
         * \param ptProjPos the projected position of the point
         * \param ptProjStiffness the uncertainty in the projected position of the point, given as the stiffness matrix
         * \param problem the problem to add the projection cost to.
         * \return true on success.
         *
         */
        virtual bool addProjectionCostFunction(double* pointData,
                                               double* poseOrientation,
                                               double* posePosition,
                                               Eigen::Vector2d const& ptProjPos,
                                               Eigen::Matrix2d const& ptProjStiffness,
                                               ceres::Problem & problem) = 0;

        /*!
         * \brief addCrossProjectionCostFunction add a cross projection between two poses (i.e. two frames seeing the same point, but the point is not explicitly instanced)
         * \param pose1Orientation the pointer to the optimizable pose 1 orientation (rotation axis).
         * \param pose1Position the pointer to the optimizable pose 1 position.
         * \param ptProj1Pos the projected position of the point in frame 1.
         * \param ptProj1Stiffness the uncertainty in the projected position of the point in frame 1, given as the stiffness matrix.
         * \param pose2Orientation the pointer to the optimizable pose 2 orientation (rotation axis).
         * \param pose2Position the pointer to the optimizable pose 2 position.
         * \param ptProj2Pos the projected position of the point in frame 2.
         * \param ptProj2Stiffness the uncertainty in the projected position of the point in frame 2, given as the stiffness matrix.
         * \param problem the problem to add the projection cost to.
         * \return true on success
         */
        virtual bool addCrossProjectionCostFunction(double* pose1Orientation,
                                                    double* pose1Position,
                                                    Eigen::Vector2d const& ptProj1Pos,
                                                    Eigen::Matrix2d const& ptProj1Stiffness,
                                                    double* pose2Orientation,
                                                    double* pose2Position,
                                                    Eigen::Vector2d const& ptProj2Pos,
                                                    Eigen::Matrix2d const& ptProj2Stiffness,
                                                    ceres::Problem & problem) = 0;

        virtual bool writeResults(ModularSBASolver* solver) = 0;
        virtual bool writeUncertainty(ModularSBASolver* solver) = 0;
        virtual void cleanup(ModularSBASolver* solver) = 0;
    };

    /*!
     * \brief The ErrorBlockLogger class represent a logger for an error block
     */
    class ValueBlockLogger {
    public:
        ValueBlockLogger();
        virtual ~ValueBlockLogger();

        virtual QVector<double> getErrors() const = 0;
        QTextStream& log(QTextStream& stream) const;

    };

    template<int nVals>
    class ParamsValsLogger : public ValueBlockLogger {

    public:
        ParamsValsLogger(double* vals) {
            _vals = vals;
        }

        virtual QVector<double> getErrors() const {

            return QVector<double>(_vals, _vals+nVals);
        }

    protected:
        double* _vals;
    };

    template<int nArgs, int errDims>
    class AutoErrorBlockLogger : public ValueBlockLogger {

    public:
        using FuncType = ceres::CostFunction*;
        using ParamsType = std::array<double*, nArgs>;
        using ErrsType = std::array<double, errDims>;

        AutoErrorBlockLogger(ceres::CostFunction* func, ParamsType const& params) {
            _func = func;
            _parameters = params;
        }

        virtual QVector<double> getErrors() const {

            ErrsType errors;

            _func->Evaluate(_parameters.data(), errors.data(), nullptr);

            return QVector<double>(errors.begin(), errors.end());
        }

    protected:

        ceres::CostFunction* _func;
        ParamsType _parameters;

    };

    /*!
     * \brief The LandmarkNode struct represent an optimizable point
     */
    struct LandmarkNode {
        qint64 lmId;
        std::array<double,3> pos;
    };

    /*!
     * \brief The PoseNode struct represent an optimizable pose (for a camera generally, but can be something else when used by a module)
     */
    struct PoseNode {
        qint64 frameId;
        std::array<double, 3> rAxis; //we go with the rotation axis representation. It is compressed, can be converted and allow to set priors more easily
        std::array<double, 3> t;
    };

    /*!
     * \brief The LeverArmNode struct store a lever arm for a combination of trajectory and platform (generally a camera).
     */
    struct LeverArmNode {
        qint64 TrajId;
        qint64 PlatformId;
        std::array<double, 3> rAxis; //we go with the rotation axis representation. It is compressed, can be converted and allow to set priors more easily
        std::array<double, 3> t;
    };

    /*!
     * \brief The TrajectoryPoseNode struct represent a full trajectory (a succession of poses with velocity in between.
     */
    struct TrajectoryPoseNode {
        double time;
        std::array<double, 3> rAxis; //rotation axis
        std::array<double, 3> t; //position
    };

    struct TrajectoryNode {
        qint64 trajId;
        std::vector<TrajectoryPoseNode> nodes;
        qint64 resultTableId;

        /*!
         * \brief getNodeForTime get the index of the node just before a set time
         * \param time the time
         * \return the index of the TrajectoryPoseNode coming just before time (no other node after comes before time), or -1 if none exist.
         */
        inline int getNodeForTime(double time) {

            for (int i = nodes.size()-1; i >= 0; i--) {
                if (nodes[i].time < time) {
                    return i;
                }
            }

            return -1;

        }
    };

    ModularSBASolver(Project* p, bool computeUncertainty = true, bool sparse = true, bool verbose = false, QObject *parent = nullptr);
    ~ModularSBASolver();

    int uncertaintySteps() const override;
    bool hasUncertaintyStep() const override;

    /*!
     * \brief addModule add a module to the solver
     * \param module the module to add
     * \return true on success, false otherwise.
     *
     * On sucess the solver take ownership of the module, and delete it once the solver is deleted.
     */
    bool addModule(SBAModule* module);

    /*!
     * \brief addProjector add a projector to the solver.
     * \param projector the projector to add
     * \return true if the projector was added, or was already in the solver, false otherwise
     */
    bool addProjector(ProjectorModule* projector);
    /*!
     * \brief assignProjectorToFrame assign a given projector to a given frame
     * \param projector the projector to use
     * \param imId the id of the frame
     * \return true if the association has been done, false otherwise.
     */
    bool assignProjectorToFrame(ProjectorModule* projector, qint64 imId);

    /*!
     * \brief getNodeForLandmark access the node for a give landmark and create it if necessary
     * \param lmId the id of the landmark
     * \param createIfMissing create the node if it does not exist yet
     * \return the landmark node
     *
     * This function will create the node if it does not exist,
     * set the node constant if the position of the landmark is fixed
     * as well as create the required prior if the position has a prior.
     */
    LandmarkNode* getNodeForLandmark(qint64 lmId, bool createIfMissing);

    /*!
     * \brief getNodeForFrame access the node for a give frame(image) and create it if necessary
     * \param imId the image id
     * \param createIfMissing create the node if it does not exist yet
     * \return the frame node
     *
     * This function will create the node if it does not exist,
     * set the node constant if the pose of the image is fixed
     * as well as create the required prior if the pose has a prior.
     */
    PoseNode* getNodeForFrame(qint64 imId, bool createIfMissing);

    /*!
     * \brief getProjectorForFrame get a projector for a given image
     * \param imId the id of the image
     * \return the projector.
     */
    ProjectorModule* getProjectorForFrame(qint64 imId);

    /*!
     * \brief getNodeForFrame access the node for a give local coordinate system and create it if necessary
     * \param lcsId the local coordinate system id
     * \param createIfMissing create the node if it does not exist yet
     * \return the local coordinate system node
     *
     * This function will create the node if it does not exist,
     * set the node constant if the pose of the local coordinate system is fixed
     * as well as create the required prior if the pose has a prior.
     */
    PoseNode* getNodeForLocalCoordinates(qint64 lcsId, bool createIfMissing);

    /*!
     * \brief getNodeForATrajectory access the node for a trajectory and create the node if necessary
     * \param trajId the id of the trajectory
     * \param createIfMissing create the node if it does not exist yet
     * \return the node for the trajectory
     *
     * Note that unlike other node creation function, this do not setup anything for the node (prior or other) when creating the node.
     * This is because trajectories are more complex objects and the responsability of setting them up is left to the modules.
     *
     * It is the responsability of the programmer to insert the module setting up trajectories before the modules using them.
     */
    TrajectoryNode* getNodeForTrajectory(qint64 trajId, bool createIfMissing);

    /*!
     * \brief getNodeForLeverArm access the node for a lever arm
     * \param camtrajId the pair of ids of the cam (first) and the trajectory (second).
     * \param createIfMissing create the node if it does not exist yet
     * \return the node for the lever arm
     */
    LeverArmNode* getNodeForLeverArm(QPair<qint64,qint64> camtrajId, bool createIfMissing);

    /*!
     * \brief getTransform2LocalFrame get the transform from the world frame (ecef, or local if local frame is used for the project).
     * \return an AffineTransform, identity if it is not defined.
     */
    StereoVision::Geometry::AffineTransform<double> getTransform2LocalFrame() const;

    inline QList<qint64> landmarksList() const {
        return _LandmarkParametersIndex.keys();
    }

    inline QList<qint64> framesList() const {
        return _frameParametersIndex.keys();
    }


    inline QList<qint64> localCoordinatesList() const {
        return _localCoordinatesParametersIndex.keys();
    }

    inline QList<qint64> trajectoriesList() const {
        return _trajectoryParametersIndex.keys();
    }

    void enableLogging(QString loggingDirPath);
    void logDatas(QString fileName);
    void addLogger(QString const& loggerName, ValueBlockLogger* logger);

protected:

    bool init() override;
    bool opt_step() override;
    bool std_step() override;
    bool writeResults() override;
    bool writeUncertainty() override;
    void cleanup() override;

    bool splitOptSteps() const override;

    bool _sparse;
    bool _verbose;
    bool _compute_marginals;
    bool _not_first_step;

    ceres::Problem _problem;
    std::vector<SBAModule*> _modules;
    QVector<ProjectorModule*> _projectors;

    QMap<QString, ValueBlockLogger*> _loggers;
    QString _loggingDir;

    QMap<qint64, int> _frameProjectorsAssociations;

    //using std vectors for the data, as these are guaranteed to be contiguous in memory.
    std::vector<LandmarkNode> _LandmarkParameters;
    QMap<qint64, int> _LandmarkParametersIndex;
    std::vector<PoseNode> _frameParameters;
    QMap<qint64, int> _frameParametersIndex;
    std::vector<PoseNode> _localCoordinatesParameters;
    QMap<qint64, int> _localCoordinatesParametersIndex;
    std::vector<TrajectoryNode> _trajectoryParameters;
    QMap<qint64, int> _trajectoryParametersIndex;
    std::vector<LeverArmNode> _leverArmParameters;
    QMap<QPair<qint64,qint64>, int> _leverArmParametersIndex;

    friend class ModularSBASolverIterationCallback;
};

class ImageAlignementSBAModule : public ModularSBASolver::SBAModule
{

public:

    ImageAlignementSBAModule();

    virtual bool init(ModularSBASolver* solver, ceres::Problem & problem) override;
    virtual bool writeResults(ModularSBASolver* solver) override;
    virtual bool writeUncertainty(ModularSBASolver* solver) override;
    virtual void cleanup(ModularSBASolver* solver) override;

protected:

    //match camera internal id to a projector module for the images
    QMap<qint64, ModularSBASolver::ProjectorModule*> _cameraProjectors;

};

class PinholdeCamProjModule : public ModularSBASolver::ProjectorModule
{

public:

    PinholdeCamProjModule(Camera* associatedCamera);
    ~PinholdeCamProjModule();

    virtual bool addProjectionCostFunction(double* pointData,
                                           double* poseOrientation,
                                           double* posePosition,
                                           Eigen::Vector2d const& ptProjPos,
                                           Eigen::Matrix2d const& ptProjStiffness,
                                           ceres::Problem & problem) override;

    virtual bool addCrossProjectionCostFunction(double* pose1Orientation,
                                                double* pose1Position,
                                                Eigen::Vector2d const& ptProj1Pos,
                                                Eigen::Matrix2d const& ptProj1Stiffness,
                                                double* pose2Orientation,
                                                double* pose2Position,
                                                Eigen::Vector2d const& ptProj2Pos,
                                                Eigen::Matrix2d const& ptProj2Stiffness,
                                                ceres::Problem & problem) override;

    bool init(ModularSBASolver* solver, ceres::Problem & problem);
    virtual bool writeResults(ModularSBASolver* solver) override;
    virtual bool writeUncertainty(ModularSBASolver* solver) override;
    virtual void cleanup(ModularSBASolver* solver) override;

protected:

    Camera* _associatedCamera;

    double _fLen; //f
    std::array<double, 2> _principalPoint; //pp
    std::array<double, 3> _radialDistortion; //k1, k2, k3
    std::array<double, 2> _tangentialDistortion; //t1, t2
    std::array<double, 2> _skewDistortion; //B1, B2



};

class TrajectoryBaseSBAModule : public ModularSBASolver::SBAModule
{

public:

    TrajectoryBaseSBAModule(double integrationTime);

    virtual bool init(ModularSBASolver* solver, ceres::Problem & problem) override;
    virtual bool writeResults(ModularSBASolver* solver) override;
    virtual bool writeUncertainty(ModularSBASolver* solver) override;
    virtual void cleanup(ModularSBASolver* solver) override;

    inline double gpsAccuracy() const
    {
        return _gpsAccuracy;
    }

    inline void setGpsAccuracy(double newGpsAccuracy)
    {
        _gpsAccuracy = newGpsAccuracy;
    }

    inline double orientAccuracy() const
    {
        return _orientAccuracy;
    }

    inline void setOrientAccuracy(double newOrientAccuracy)
    {
        _orientAccuracy = newOrientAccuracy;
    }

    inline double accAccuracy() const
    {
        return _accAccuracy;
    }

    inline void setAccAccuracy(double newAccAccuracy)
    {
        _accAccuracy = newAccAccuracy;
    }

    inline double gyroAccuracy() const
    {
        return _gyroAccuracy;
    }

    inline void setGyroAccuracy(double newGyroAccuracy)
    {
        _gyroAccuracy = newGyroAccuracy;
    }

protected:

    double _integrationTime;

    double _gpsAccuracy;
    double _orientAccuracy;

    double _accAccuracy;
    double _gyroAccuracy;

    std::array<double,3> _gravity;

};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_MODULARSBASOLVER_H
