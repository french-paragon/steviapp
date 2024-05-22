#ifndef STEREOVISIONAPP_TRAJECTORY_H
#define STEREOVISIONAPP_TRAJECTORY_H

#include <QObject>

#include "./project.h"
#include "./georeferenceddatablockinterface.h"

#include "../geo/localframes.h"
#include "../vision/indexed_timed_sequence.h"

#include <StereoVision/geometry/core.h>
#include <StereoVision/geometry/rotations.h>

namespace StereoVisionApp {

/*!
 * \brief The Trajectory class encode informations about a trajecory stored in a csv file
 */
class Trajectory : public DataBlock, public GeoReferencedDataBlockInterface
{
    Q_OBJECT
    Q_INTERFACES(StereoVisionApp::GeoReferencedDataBlockInterface)
public:

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

    /*!
     * \brief loadPositionSequence load an IndexedTimeSequence for the orientation (in local topographic frames).
     * \return optionnaly a TimeCartesianSequence
     */
    std::optional<TimeCartesianSequence> loadOrientationSequence() const;

    /*!
     * \brief loadPositionSequence load an IndexedTimeSequence for the position (converted to ECEF if a geodetic conversion is defined)
     * \return optionnaly a TimeCartesianSequence
     */
    std::optional<TimeCartesianSequence> loadPositionSequence() const;

    /*!
     * \brief loadTrajectoryPathInProjectLocalFrame load the position data, and apply the conversion from ECEF to project local frame on the fly.
     * \return a vector of Eigen::Vector3f
     *
     * This function is used mostly to display the trajectory in view widgets
     */
    std::vector<Eigen::Vector3f> loadTrajectoryPathInProjectLocalFrame() const;


    /*!
     * \brief loadTrajectorySequence load the trajectory (as sequence of pose from plateform to ECEF if a geodetic conversion is defined)
     * \return optionaly a TimeTrajectorySequence
     *
     * If a plateform is defined (boresight and lever arm), the trajectory represent the trajectory of the plateform, else the trajectory represent the trajectory of the sensor
     */
    std::optional<TimeTrajectorySequence> loadTrajectorySequence() const;

    /*!
     * \brief loadTrajectoryProjectLocalFraneSequence load the trajectory data, and apply the conversion from ECEF to project local frame on the fly.
     * \return optionaly a TimeTrajectorySequence
     */
    std::optional<TimeTrajectorySequence> loadTrajectoryProjectLocalFrameSequence() const;

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

Q_SIGNALS:
    void trajectoryDataChanged();

protected:

    /*!
     * \brief loadPositionData load the position data (including geodetic conversion if defined properly)
     * \return a vector of time position blocks
     */
    std::vector<TimeCartesianBlock> loadPositionData() const;

    /*!
     * \brief loadPositionRawData load the position raw data (i.e. without any geodetic conversion).
     * \return a vector of time position blocks
     */
    std::vector<TimeCartesianBlock> loadPositionRawData() const;

    /*!
     * \brief loadPositionRawData load the orientation raw data (i.e. without any conversion from local topometric frame).
     * \return a vector of time orientation blocks (represented as axis angles)
     */
    std::vector<TimeCartesianBlock> loadOrientationRawData() const;

    /*!
     * \brief loadPositionData load the trajectory (including geodetic conversion of the position, if defined properly, and conversion of the orientation from local topometric frame)
     * \return a vector of time trajectory blocks (represented as axis angles)
     */
    std::vector<TimeTrajectoryBlock> loadTrajectoryData() const;

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
