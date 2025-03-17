#ifndef STEREOVISIONAPP_TRAJECTORY_H
#define STEREOVISIONAPP_TRAJECTORY_H

#include <QObject>

#include "./floatparameter.h"
#include "./project.h"
#include "./georeferenceddatablockinterface.h"

#include "../geo/localframes.h"
#include "../vision/indexed_timed_sequence.h"

#include "../utils/statusoptionalreturn.h"

#include <StereoVision/geometry/core.h>
#include <StereoVision/geometry/rotations.h>

namespace StereoVisionApp {

class DataTable;

/*!
 * \brief The Trajectory class encode informations about a trajecory stored in a csv file
 */
class Trajectory : public DataBlock, public GeoReferencedDataBlockInterface
{
    Q_OBJECT
    Q_INTERFACES(StereoVisionApp::GeoReferencedDataBlockInterface)

public:

    static const char* OptDataTimeHeader;
    static const char* OptDataPosXHeader;
    static const char* OptDataPosYHeader;
    static const char* OptDataPosZHeader;
    static const char* OptDataRotXHeader;
    static const char* OptDataRotYHeader;
    static const char* OptDataRotZHeader;

    enum Representation {
        RepText,
        RepBinary
    };

    enum BinaryDataType {
        SignedInt16,
        UnsignedInt16,
        SignedInt32,
        UnsignedInt32,
        SignedInt64,
        UnsignedInt64,
        FloatSingle,
        FloatDouble
    };

    enum VariableType {
        Position,
        Position2D,
        Speed,
        Acceleration,
        Orientation,
        AngularSpeed,
        AngularAcceleration
    };
    Q_ENUM(VariableType)

    enum TopocentricConvention {
        NED = Geo::TopocentricConvention::NED ,
        ENU = Geo::TopocentricConvention::ENU,
        NWU = Geo::TopocentricConvention::NWU
    };
    Q_ENUM(TopocentricConvention)

    enum AngleRepresentation {
        AxisAngle,
        Quaternion,
        EulerXYZ,
        EulerZYX
    };
    Q_ENUM(AngleRepresentation)

    enum AngleUnits {
        Radians,
        Degree,
        Gon
    };
    Q_ENUM(AngleUnits)

    enum Axis {
        T = 3,
        X = 0,
        Y = 1,
        Z = 2,
        W = 4
    };
    Q_ENUM(Axis)

    Trajectory(Project* parent = nullptr);

    using PoseType = StereoVision::Geometry::RigidBodyTransform<double>;

    using TimeCartesianSequence = IndexedTimeSequence<Eigen::Vector3d, double>;
    using TimeTrajectorySequence = IndexedTimeSequence<PoseType, double>;

    using TimeCartesianBlock = TimeCartesianSequence::TimedElement;
    using TimeTrajectoryBlock = TimeTrajectorySequence::TimedElement;

    static StatusOptionalReturn<std::vector<TimeTrajectoryBlock>> guidedSubsampleTrajectory
    (std::vector<TimeTrajectoryBlock> const& trajectory,
     std::vector<TimeTrajectoryBlock> const& guide);

    /*!
     * \brief loadAngularSpeedSequence load an IndexedTimeSequence for the angular speed (in body frame).
     * \return optionnaly a TimeCartesianSequence
     */
    StatusOptionalReturn<TimeCartesianSequence> loadAngularSpeedSequence() const;

    /*!
     * \brief loadAccelerationSequence load an IndexedTimeSequence for the acceleration (in body frame)
     * \return optionnaly a TimeCartesianSequence
     */
    StatusOptionalReturn<TimeCartesianSequence> loadAccelerationSequence() const;

    /*!
     * \brief loadPositionSequence load an IndexedTimeSequence for the orientation (in local topographic frames).
     * \return optionnaly a TimeCartesianSequence
     */
    StatusOptionalReturn<TimeCartesianSequence> loadOrientationSequence() const;

    /*!
     * \brief loadPositionSequence load an IndexedTimeSequence for the position (converted to ECEF if a geodetic conversion is defined)
     * \return optionnaly a TimeCartesianSequence
     */
    StatusOptionalReturn<TimeCartesianSequence> loadPositionSequence() const;

    /*!
     * \brief loadTrajectoryPathInProjectLocalFrame load the position data, and apply the conversion from ECEF to project local frame on the fly.
     * \return a vector of Eigen::Vector3f
     *
     * This function is used mostly to display the trajectory in view widgets
     */
    StatusOptionalReturn<std::vector<Eigen::Vector3f>> loadTrajectoryPathInProjectLocalFrame() const;
    /*!
     * \brief loadTrajectoryInProjectLocalFrame load the pose data, and apply the conversion from ECEF to project local frame on the fly.
     * \return a vector of StereoVision::Geometry::AffineTransform<float>
     *
     * This function is used mostly to display the trajectory in view widgets
     */
    StatusOptionalReturn<std::vector<StereoVision::Geometry::AffineTransform<float>>> loadTrajectoryInProjectLocalFrame(bool optimized = false) const;


    /*!
     * \brief loadTrajectorySequence load the trajectory (as sequence of pose from plateform to ECEF if a geodetic conversion is defined)
     * \return optionaly a TimeTrajectorySequence
     *
     * If a plateform is defined (boresight and lever arm), the trajectory represent the trajectory of the plateform, else the trajectory represent the trajectory of the sensor
     */
    StatusOptionalReturn<TimeTrajectorySequence> loadTrajectorySequence() const;

    /*!
     * \brief loadTrajectoryProjectLocalFraneSequence load the trajectory data, and apply the conversion from ECEF to project local frame on the fly.
     * \return optionaly a TimeTrajectorySequence
     */
    StatusOptionalReturn<TimeTrajectorySequence> loadTrajectoryProjectLocalFrameSequence() const;

    /*!
     * \brief optimizedTrajectory return the optimized trajectory, if it is set. The optimized trajectory will be in the project local frame
     * \param subsample if true, use the initial trajectory to subsample the optimized trajectory.
     * \return optionaly a TimeTrajectorySequence
     */
    StatusOptionalReturn<TimeTrajectorySequence> optimizedTrajectory(bool subsample = false) const;
    /*!
     * \brief optimizedTrajectoryECEF return the optimized trajectory, if it is set. The optimized trajectory will be in the ECEF frame
     * \param subsample if true, use the initial trajectory to subsample the optimized trajectory.
     * \return optionaly a TimeTrajectorySequence
     */
    StatusOptionalReturn<TimeTrajectorySequence> optimizedTrajectoryECEF(bool subsample = false) const;

    virtual bool geoReferenceSupportActive() const override;
    virtual Eigen::Array<float,3, Eigen::Dynamic> getLocalPointsEcef() const override;
    virtual QString getCoordinateReferenceSystemDescr(int role = DefaultCRSRole) const override;

    inline QString positionFile() const {
        return _positionFile;
    }
    void setPositionFile(QString const& path);

    inline QString positionEpsg() const {
        return _positionDefinition.crs_epsg;
    }
    void setPositionEpsg(QString const& epsg);

    inline double positionTimeScale() {
        return _positionDefinition.timeScale;
    }

    void setPositionTimeScale(double scale);

    inline double positionTimeDelta() {
        return _positionDefinition.timeDelta;
    }

    void setPositionTimeDelta(double delta);

    inline int positionColumns(Axis axis) const {
        switch (axis) {
        case T:
            return _positionDefinition.pos_t_col;
        case X:
            return _positionDefinition.pos_x_col;
        case Y:
            return _positionDefinition.pos_y_col;
        case Z:
            return _positionDefinition.pos_z_col;
        default:
            return -1;
        }
    }
    void setPositionColumn(Axis axis, int col);

    inline QString accelerometerFile() const {
        return _accelerationFile;
    }
    void setAccelerometerFile(QString const& path);

    inline double accelerometerTimeScale() {
        return _accelerationDefinition.timeScale;
    }

    void setAccelerometerTimeScale(double scale);

    inline double accelerometerTimeDelta() {
        return _accelerationDefinition.timeDelta;
    }

    inline void setAccelerometerMounting(StereoVision::Geometry::AffineTransform<double> mounting) {
        _accelerometerMounting = mounting;
    }

    inline StereoVision::Geometry::AffineTransform<double> accelerometerMounting() const {
        return _accelerometerMounting;
    }

    void setAccelerometerTimeDelta(double delta);

    inline int accelerometerColumns(Axis axis) const {
        switch (axis) {
        case T:
            return _accelerationDefinition.acc_t_col;
        case X:
            return _accelerationDefinition.acc_x_col;
        case Y:
            return _accelerationDefinition.acc_y_col;
        case Z:
            return _accelerationDefinition.acc_z_col;
        default:
            return -1;
        }
    }
    void setAccelerometerColumn(Axis axis, int col);

    bool estAccelerometerBias() const;
    bool estAccelerometerScale() const;

    void setEstAccelerometerBias(bool estimate);
    void setEstAccelerometerScale(bool estimate);

    floatParameter optAccelerometerBiasX() const;
    floatParameter optAccelerometerBiasY() const;
    floatParameter optAccelerometerBiasZ() const;

    void setOptAccelerometerBiasX(floatParameter const& biasX);
    void setOptAccelerometerBiasY(floatParameter const& biasY);
    void setOptAccelerometerBiasZ(floatParameter const& biasZ);

    floatParameter optAccelerometerScaleX() const;
    floatParameter optAccelerometerScaleY() const;
    floatParameter optAccelerometerScaleZ() const;

    void setOptAccelerometerScaleX(floatParameter const& scale);
    void setOptAccelerometerScaleY(floatParameter const& scale);
    void setOptAccelerometerScaleZ(floatParameter const& scale);


    inline QString orientationFile() const {
        return _orientationFile;
    }
    void setOrientationFile(QString const& path);

    inline TopocentricConvention orientationTopocentricConvention() const {
        return _orientationDefinition.topocentric_convention;
    }
    void setOrientationTopocentricConvention(TopocentricConvention convention);
    inline void setOrientationTopocentricConvention(int convention) {
        return setOrientationTopocentricConvention(static_cast<TopocentricConvention>(convention));
    }

    inline AngleRepresentation orientationAngleRepresentation() const {
        return _orientationDefinition.angle_representation;
    }
    void setOrientationAngleRepresentation(AngleRepresentation representation);
    inline void setOrientationAngleRepresentation(int representation) {
        return setOrientationAngleRepresentation(static_cast<AngleRepresentation>(representation));
    }

    inline AngleUnits orientationAngleUnits() const {
        return _orientationDefinition.angleUnit;
    }
    void setOrientationAngleUnits(AngleUnits units);
    inline void setOrientationAngleUnits(int units) {
        return setOrientationAngleUnits(static_cast<AngleUnits>(units));
    }

    inline double orientationTimeScale() {
        return _orientationDefinition.timeScale;
    }

    void setOrientationTimeScale(double scale);

    inline double orientationTimeDelta() {
        return _orientationDefinition.timeDelta;
    }

    void setOrientationTimeDelta(double delta);

    inline int orientationColumns(Axis axis) const {
        switch (axis) {
        case T:
            return _orientationDefinition.orient_t_col;
        case X:
            return _orientationDefinition.orient_x_col;
        case Y:
            return _orientationDefinition.orient_y_col;
        case Z:
            return _orientationDefinition.orient_z_col;
        case W:
            return _orientationDefinition.orient_w_col;
        }
    }
    void setOrientationColumn(Axis axis, int col);

    inline int orientationSign(Axis axis) {
        switch (axis) {
        case T:
            return 1;
        case X:
            return (_orientationDefinition.sign_x > 0) ? 1 : -1;
        case Y:
            return (_orientationDefinition.sign_y > 0) ? 1 : -1;
        case Z:
            return (_orientationDefinition.sign_z > 0) ? 1 : -1;
        case W:
            return (_orientationDefinition.sign_w > 0) ? 1 : -1;
        }
    }
    void setOrientationSign(Axis axis, int sign);


    inline QString gyroFile() const {
        return _angularSpeedFile;
    }
    void setGyroFile(QString const& path);


    inline AngleRepresentation gyroAngleRepresentation() const {
        return _angularSpeedDefinition.angle_representation;
    }
    void setGyroAngleRepresentation(AngleRepresentation representation);
    inline void setGyroAngleRepresentation(int representation) {
        return setGyroAngleRepresentation(static_cast<AngleRepresentation>(representation));
    }

    inline AngleUnits gyroAngleUnits() const {
        return _angularSpeedDefinition.angleUnit;
    }
    void setGyroAngleUnits(AngleUnits units);
    inline void setGyroAngleUnits(int units) {
        return setGyroAngleUnits(static_cast<AngleUnits>(units));
    }

    inline double gyroTimeScale() {
        return _angularSpeedDefinition.timeScale;
    }

    void setGyroTimeScale(double scale);

    inline double gyroTimeDelta() {
        return _angularSpeedDefinition.timeDelta;
    }

    void setGyroTimeDelta(double delta);

    inline int gyroColumns(Axis axis) const {
        switch (axis) {
        case T:
            return _angularSpeedDefinition.w_t_col;
        case X:
            return _angularSpeedDefinition.w_x_col;
        case Y:
            return _angularSpeedDefinition.w_y_col;
        case Z:
            return _angularSpeedDefinition.w_z_col;
        case W:
            return _angularSpeedDefinition.w_w_col;
        }
    }
    void setGyroColumn(Axis axis, int col);

    inline int gyroSign(Axis axis) {
        switch (axis) {
        case T:
            return 1;
        case X:
            return (_angularSpeedDefinition.sign_x > 0) ? 1 : -1;
        case Y:
            return (_angularSpeedDefinition.sign_y > 0) ? 1 : -1;
        case Z:
            return (_angularSpeedDefinition.sign_z > 0) ? 1 : -1;
        case W:
            return (_angularSpeedDefinition.sign_w > 0) ? 1 : -1;
        }
    }
    void setGyroSign(Axis axis, int sign);

    inline void setGyroMounting(StereoVision::Geometry::AffineTransform<double> mounting) {
        _gyroMounting = mounting;
    }

    inline StereoVision::Geometry::AffineTransform<double> gyroMounting() const {
        return _gyroMounting;
    }

    bool estGyroBias() const;
    bool estGyroScale() const;

    void setEstGyroBias(bool estimate);
    void setEstGyroScale(bool estimate);

    floatParameter optGyroBiasX() const;
    floatParameter optGyroBiasY() const;
    floatParameter optGyroBiasZ() const;

    void setOptGyroBiasX(floatParameter const& biasX);
    void setOptGyroBiasY(floatParameter const& biasY);
    void setOptGyroBiasZ(floatParameter const& biasZ);

    floatParameter optGyroScaleX() const;
    floatParameter optGyroScaleY() const;
    floatParameter optGyroScaleZ() const;

    void setOptGyroScaleX(floatParameter const& scale);
    void setOptGyroScaleY(floatParameter const& scale);
    void setOptGyroScaleZ(floatParameter const& scale);

    double getPreIntegrationTime() const;
    void setPreIntegrationTime(double time);

    double getGpsAccuracy() const;
    double getAngularAccuracy() const;
    double getGyroAccuracy() const;
    double getAccAccuracy() const;

    void setGpsAccuracy(double accuracy);
    void setAngularAccuracy(double accuracy);
    void setGyroAccuracy(double accuracy);
    void setAccAccuracy(double accuracy);

    DataTable* getOptimizedDataTable();
    bool hasInitialTrajectory() const;
    bool hasOptimizedTrajectory() const;

    int accelerometerId() const;
    void setAccelerometerId(int newAccelerometerId);

    int gyroId() const;
    void setGyroId(int newGyroId);

Q_SIGNALS:
    void trajectoryDataChanged();
    void optimizedTrajectoryDataChanged();

    void gpsAccuracyChanged();
    void angularAccuracyChanged();
    void gyroAccuracyChanged();
    void accAccuracyChanged();

    void accelerometerBiasXChanged();
    void accelerometerBiasYChanged();
    void accelerometerBiasZChanged();

    void accelerometerScaleXChanged();
    void accelerometerScaleYChanged();
    void accelerometerScaleZChanged();

    void estimateAccelerometerBiasChanged();
    void estimateAccelerometerScaleChanged();


    void gyroBiasXChanged();
    void gyroBiasYChanged();
    void gyroBiasZChanged();

    void gyroScaleXChanged();
    void gyroScaleYChanged();
    void gyroScaleZChanged();

    void estimateGyroBiasChanged();
    void estimateGyroScaleChanged();

    void accelerometerIdChanged();
    void gyroIdChanged();

    void preIntegrationTimeChanged();

protected:


    /*!
     * \brief loadAngularSpeedRawData load the angular speed data (in body frame)
     * \return the data as TimeCartesianBlocks representing an angular speed, or empty vector if failed to load
     */
    StatusOptionalReturn<std::vector<TimeCartesianBlock>> loadAngularSpeedRawData() const;

    /*!
     * \brief loadAccelerationRawData load the acceleration data (in body frame)
     * \return the data as TimeCartesianBlocks, or empty vector if failed to load
     */
    StatusOptionalReturn<std::vector<TimeCartesianBlock>> loadAccelerationRawData() const;

    /*!
     * \brief loadPositionData load the position data (including geodetic conversion if defined properly)
     * \return a vector of time position blocks
     */
    StatusOptionalReturn<std::vector<TimeCartesianBlock>> loadPositionData() const;

    /*!
     * \brief loadPositionRawData load the position raw data (i.e. without any geodetic conversion).
     * \return a vector of time position blocks
     */
    StatusOptionalReturn<std::vector<TimeCartesianBlock>> loadPositionRawData() const;

    /*!
     * \brief loadOrientationRawData load the orientation raw data (i.e. without any conversion from local topometric frame).
     * \return a vector of time orientation blocks (represented as axis angles)
     */
    StatusOptionalReturn<std::vector<TimeCartesianBlock>> loadOrientationRawData() const;

    /*!
     * \brief loadPositionData load the trajectory (including geodetic conversion of the position, if defined properly, and conversion of the orientation from local topometric frame)
     * \return a vector of time trajectory blocks (represented as axis angles)
     */
    StatusOptionalReturn<std::vector<TimeTrajectoryBlock>> loadTrajectoryData() const;

    QJsonObject encodeJson() const override;
    void configureFromJson(QJsonObject const& data) override;

    void extendDataModel();

    QStringList _separators;
    QString _commentPattern;

    struct PlateformDefinition {

        Eigen::Vector3d boresight;
        Eigen::Vector3d leverArm;

    } _plateformDefinition;

    struct OrientationDefinition {

        TopocentricConvention topocentric_convention;
        AngleRepresentation angle_representation;
        AngleUnits angleUnit; //unit for the angles, if input euler angles

        double timeDelta;
        double timeScale;

        int orient_t_col;
        int orient_x_col;
        int orient_y_col;
        int orient_z_col;
        int orient_w_col;

        int orient_x_col_std;
        int orient_y_col_std;
        int orient_z_col_std;
        int orient_w_col_std;

        BinaryDataType orient_t_col_type;
        BinaryDataType orient_x_col_type;
        BinaryDataType orient_y_col_type;
        BinaryDataType orient_z_col_type;
        BinaryDataType orient_w_col_type;

        BinaryDataType orient_x_col_std_type;
        BinaryDataType orient_y_col_std_type;
        BinaryDataType orient_z_col_std_type;
        BinaryDataType orient_w_col_std_type;

        int binary_header_lenght;
        int binary_line_lenght;

        int8_t sign_x;
        int8_t sign_y;
        int8_t sign_z;
        int8_t sign_w;


    } _orientationDefinition;

    QString _orientationFile;

    struct PositionDefinition {

        QString crs_epsg;

        double timeDelta;
        double timeScale;

        int pos_t_col;
        int pos_x_col;
        int pos_y_col;
        int pos_z_col;

    } _positionDefinition;

    QString _positionFile;

    struct AccelerometerDefinition {

        double timeDelta;
        double timeScale;

        int acc_t_col;
        int acc_x_col;
        int acc_y_col;
        int acc_z_col;

    } _accelerationDefinition;

    struct AccelerometerParameters {

        bool _estimates_bias;
        bool _estimates_gain;

        floatParameter _o_biasX;
        floatParameter _o_biasY;
        floatParameter _o_biasZ;
        floatParameter _o_scaleX;
        floatParameter _o_scaleY;
        floatParameter _o_scaleZ;

    } _accelerometerParameters;

    StereoVision::Geometry::AffineTransform<double> _accelerometerMounting;
    QString _accelerationFile;

    struct AngularSpeedDefinition {

        AngleRepresentation angle_representation;
        AngleUnits angleUnit; //unit for the angles, if input euler angles

        double timeDelta;
        double timeScale;

        int w_t_col;
        int w_x_col;
        int w_y_col;
        int w_z_col;
        int w_w_col;

        int8_t sign_x;
        int8_t sign_y;
        int8_t sign_z;
        int8_t sign_w;

    } _angularSpeedDefinition;

    struct AngularSpeedParameters {

        bool _estimates_bias;
        bool _estimates_gain;

        floatParameter _o_biasX;
        floatParameter _o_biasY;
        floatParameter _o_biasZ;
        floatParameter _o_scaleX;
        floatParameter _o_scaleY;
        floatParameter _o_scaleZ;

    } _angularSpeedParameters;

    StereoVision::Geometry::AffineTransform<double> _gyroMounting;
    QString _angularSpeedFile;

    mutable std::optional<TimeTrajectorySequence> _trajectoryCache;

    int _accelerometerId;
    int _gyroId;

    double _gpsAccuracy;
    double _angularAccuracy;
    double _gyroAccuracy;
    double _accAccuracy;

    /*!
     * \brief _opt_integration_time store the pre-integration time for optimization.
     * _opt_integration_time < 0 imply to use the app global pre-integration time
     */
    double _opt_preintegration_time;

    DataTable* _optimizedTrajectoryData;
};

class TrajectoryFactory : public DataBlockFactory
{
    Q_OBJECT
public:
    explicit TrajectoryFactory(QObject* parent = nullptr);

    virtual QString TypeDescrName() const;
    virtual FactorizableFlags factorizable() const;
    virtual DataBlock* factorizeDataBlock(Project *parent = nullptr) const;

    virtual QString itemClassName() const;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_TRAJECTORY_H
