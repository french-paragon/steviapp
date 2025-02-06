#ifndef STEREOVISIONAPP_MODULARSBASOLVER_H
#define STEREOVISIONAPP_MODULARSBASOLVER_H

#include "./sparsesolverbase.h"
#include "./sbagraphreductor.h"

#include <optional>

#include <ceres/ceres.h>

#include <QVector>
#include <QMap>

#include <Eigen/Core>
#include <StereoVision/geometry/rotations.h>

#include "../vision/indexed_timed_sequence.h"

class QTextStream;

namespace StereoVisionApp {

class Camera;
class GenericSBAGraphReductor;

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

        virtual QString moduleName() const = 0;

        virtual bool addGraphReductorVariables(Project *currentProject, GenericSBAGraphReductor* graphReductor) = 0;
        virtual bool addGraphReductorObservations(Project *currentProject, GenericSBAGraphReductor* graphReductor) = 0;

        /*!
         * \brief setupParameters is meant to setup the different parameters that the module is supposed to provide
         * \param solver the modular solver
         * \return false in case of error, true otherwise.
         */
        virtual bool setupParameters(ModularSBASolver* solver) = 0;
        /*!
         * \brief init initialize the part of the optimization problem this module is supposed to deal with. At this point, parameters are expected to be setup in the solver, but not yet in the problem.
         * \param solver the modular solver
         * \param problem the ceres problem
         * \return false in case of error, true otherwise.
         */
        virtual bool init(ModularSBASolver* solver, ceres::Problem & problem) = 0;
        /*!
         * \brief writeResults write the results back to the project
         * \param solver the modular solver
         * \return false in case of error, true otherwise.
         */
        virtual bool writeResults(ModularSBASolver* solver) = 0;
        /*!
         * \brief writeUncertainty write uncertainty back to the project
         * \param solver the modular solver
         * \return false in case of error, true otherwise.
         */
        virtual bool writeUncertainty(ModularSBASolver* solver) = 0;
        /*!
         * \brief cleanup cleanup after optimization
         * \param solver the modular solver
         */
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


        virtual bool init(ModularSBASolver* solver, ceres::Problem & problem) = 0;

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
                                               ceres::Problem & problem,
                                               StereoVision::Geometry::RigidBodyTransform<double> const& offset = StereoVision::Geometry::RigidBodyTransform<double>(Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero()),
                                               double* leverArmOrientation = nullptr,
                                               double* leverArmPosition = nullptr) = 0;

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

        AutoErrorBlockLogger(ceres::CostFunction* func, ParamsType const& params, bool manageFunc = false) {
            _func = func;
            _manageFunc = manageFunc;
            _parameters = params;
        }

        virtual ~AutoErrorBlockLogger() {
            if (_manageFunc) {
                delete _func;
            }
        }

        virtual QVector<double> getErrors() const {

            ErrsType errors;

            _func->Evaluate(_parameters.data(), errors.data(), nullptr);

            return QVector<double>(errors.begin(), errors.end());
        }

    protected:

        ceres::CostFunction* _func;
        bool _manageFunc;
        ParamsType _parameters;

    };

    /*!
     * \brief The PositionNode struct represent an optimizable point
     */
    struct PositionNode {
        qint64 datablockId;
        std::array<double,3> pos;
    };

    /*!
     * \brief The PoseNode struct represent an optimizable pose (for a camera generally, but can be something else when used by a module)
     *
     * The pose node represent the transformation from the datablock to the reference (either mapping frame or trajectory).
     */
    struct PoseNode {
        qint64 datablockId;
        std::array<double, 3> rAxis; //we go with the rotation axis representation. It is compressed, can be converted and allow to set priors more easily
        std::array<double, 3> t;
        std::optional<qint64> trajectoryId; //indicate that the pose is relative to a trajectory
        std::optional<double> time; //indicate that the pose is for a precise point in time
    };

    /*!
     * \brief The LeverArmNode struct store a lever arm for a combination of trajectory and platform (generally a camera).
     *
     * Unlike PoseNode, the lever arm node represent a transformation from body to sensor, as this is how lever arm are usually expressed
     */
    struct LeverArmNode {
        qint64 TrajId;
        qint64 PlatformId;
        std::array<double, 3> rAxis; //we go with the rotation axis representation. It is compressed, can be converted and allow to set priors more easily
        std::array<double, 3> t;
    };

    /*!
     * \brief The TrajectoryPoseNode struct represent an element in a trajectory
     */
    struct TrajectoryPoseNode {
        double time;
        std::array<double, 3> rAxis; //rotation axis
        std::array<double, 3> t; //position
    };

    /*!
     * \brief The TrajectoryNode struct represent a full trajectory (a sucession of TrajectoryPoseNode)
     */
    struct TrajectoryNode {
        qint64 trajId;
        std::vector<TrajectoryPoseNode> nodes;
        IndexedTimeSequence<StereoVision::Geometry::RigidBodyTransform<double>, double> initialTrajectory;

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
     * \brief assignProjectorToCamera assign a given projector to a given camera
     * \param projector the projector to use
     * \param camId the id of the camera
     * \return true if the association has been done, false otherwise.
     */
    bool assignProjectorToCamera(ProjectorModule* projector, qint64 camId);

    PositionNode* getPositionNode(qint64 datablockId);
    PoseNode* getPoseNode(qint64 datablockId);

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
    PositionNode* getNodeForLandmark(qint64 lmId, bool createIfMissing);

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
     * \brief getProjectorForFrame get a projector corresponding to a given camera
     * \param camId the id of the camera
     * \return the projector.
     */
    ProjectorModule* getProjectorForCamera(qint64 camId);

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
     * \brief getNodeForMounting access the node for a give mounting set and create it if necessary
     * \param mountingId the mounting id
     * \param createIfMissing create the node if it does not exist yet
     * \return the mounting node
     *
     * This function will create the node if it does not exist,
     * set the node constant if the pose of the mounting is fixed
     * as well as create the required prior if the pose has a prior.
     */
    PoseNode* getNodeForMounting(qint64 mountingId, bool createIfMissing);

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

    inline QList<qint64> pointsList() const {
        return _pointsParametersIndex.keys();
    }

    inline QList<qint64> posesList() const {
        return _poseParametersIndex.keys();
    }

    inline QList<qint64> trajectoriesList() const {
        return _trajectoryParametersIndex.keys();
    }

    void enableLogging(QString loggingDirPath);
    /*!
     * \brief logDatas write all the data logger to a specific file
     * \param fileName the filename to log to (will be created in the logging dir).
     */
    void logDatas(QString fileName);
    /*!
     * \brief logMessage if logging is enabled, log a message to the default log file.
     * \param message the message to log
     */
    void logMessage(QString message);
    void addLogger(QString const& loggerName, ValueBlockLogger* logger);

    /*!
     * \brief itemIsObservable indicate if an item is observable (according to the current observability graph in the SBA solver
     * \param itemId the item id
     * \return true if the item is considered to be observable
     *
     * Note that the state of observability depends on the state of the observability graph.
     * This method is mainly aimed at being used in the SBAModules init function, to decide whether to include an item or not.
     * The observability graph will be cleared, refilled with the addGraphReductorVariables and
     *  addGraphReductorObservations methods of the SBAModules, and then reduced before the init methods
     *  of the SBA modules are called.
     *
     */
    bool itemIsObservable(qint64 itemId) const;

protected:

    bool init() override;
    bool initManagedParameters();


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

    GenericSBAGraphReductor _observabilityGraph;

    ceres::Problem _problem;
    std::vector<SBAModule*> _modules;
    QVector<ProjectorModule*> _projectors;

    QMap<QString, ValueBlockLogger*> _loggers;
    QString _loggingDir;
    QString _default_logging_file;

    QMap<qint64, int> _cameraProjectorsAssociations;
    QMap<qint64, int> _frameProjectorsAssociations;

    //using std vectors for the data, as these are guaranteed to be contiguous in memory.
    std::vector<PositionNode> _pointsParameters;
    QMap<qint64, int> _pointsParametersIndex;

    std::vector<PoseNode> _poseParameters;
    QMap<qint64, int> _poseParametersIndex;

    std::vector<TrajectoryNode> _trajectoryParameters;
    QMap<qint64, int> _trajectoryParametersIndex;

    std::vector<LeverArmNode> _leverArmParameters;
    QMap<QPair<qint64,qint64>, int> _leverArmParametersIndex;

    friend class ModularSBASolverIterationCallback;
};

/*!
 * \brief The SBASolverModulesInterface class represent an interface to manager modules for the ModularSBASolver at the application level
 *
 * The class allows to register and unregister modules, as well as accessing them
 */
class SBASolverModulesInterface : public QObject {
    Q_OBJECT
public:

    static const char* AppInterfaceName;

    using SBASolverModuleFactory = std::function<ModularSBASolver::SBAModule*(ModularSBASolver* solver)>;
    using SBASolverProjectorModuleFactory = std::function<ModularSBASolver::ProjectorModule*(ModularSBASolver* solver)>;

    SBASolverModulesInterface(QObject* parent = nullptr);

    /*!
     * \brief registerSBAModule register a SBA module, if a module with the same name already exist, it will be removed.
     * \param name the name of the module
     * \param moduleFactory the factory to create modules of the sort.
     */
    inline void registerSBAModule(QString const& name, SBASolverModuleFactory const& moduleFactory) {
        _modulesRegister.insert(name, moduleFactory);
    }

    /*!
     * \brief buildSBAModule build a given SBA module
     * \param name the name of the module
     * \return a pointer to the module, or nullptr if no module could be created.
     */
    inline ModularSBASolver::SBAModule* buildSBAModule(QString const& name, ModularSBASolver* solver) const {
        if (!_modulesRegister.contains(name)) {
            return nullptr;
        }

        return _modulesRegister[name](solver);
    }

    /*!
     * \brief clearSBAModule remove a factory for a certain module
     * \param name the name of the module
     */
    inline void clearSBAModule(QString const& name) {
        _modulesRegister.remove(name);
    }

    /*!
     * \brief installedSBAModules list the installed sba modules
     * \return the list of modules names
     */
    inline QStringList installedSBAModules() const {
        return _modulesRegister.keys();
    }

    inline bool hasSBAModule(QString const& name) const {
        return _modulesRegister.contains(name);
    }

    /*!
     * \brief registerProjectorModule register a projector module, if a module with the same name already exist, it will be removed.
     * \param name the name of the module
     * \param moduleFactory the factory to create modules of the sort.
     */
    inline void registerProjectorModule(QString const& name, SBASolverProjectorModuleFactory const& moduleFactory) {
        _projectorsRegister.insert(name, moduleFactory);
    }

    /*!
     * \brief buildProjectorModule build a given projector module
     * \param name the name of the module
     * \return a pointer to the module, or nullptr if no module could be created.
     */
    inline ModularSBASolver::ProjectorModule* buildProjectorModule(QString const& name, ModularSBASolver* solver) const {
        if (!_projectorsRegister.contains(name)) {
            return nullptr;
        }

        return _projectorsRegister[name](solver);
    }

    /*!
     * \brief clearProjectorModule remove a factory for a certain module
     * \param name the name of the module
     */
    inline void clearProjectorModule(QString const& name) {
        _projectorsRegister.remove(name);
    }

    /*!
     * \brief installedProjectorModules list the installed projector modules
     * \return the list of modules names
     */
    inline QStringList installedProjectorModules() const {
        return _projectorsRegister.keys();
    }

    inline bool hasProjectorModule(QString const& name) const {
        return _projectorsRegister.contains(name);
    }

protected:

    QMap<QString, SBASolverModuleFactory> _modulesRegister;
    QMap<QString, SBASolverProjectorModuleFactory> _projectorsRegister;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_MODULARSBASOLVER_H
