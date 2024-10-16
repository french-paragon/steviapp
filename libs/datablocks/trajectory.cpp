#include "trajectory.h"

#include <QFile>
#include <QRegularExpression>
#include <QJsonObject>
#include <QJsonArray>

#include <QMetaEnum>

#include "geo/localframes.h"

#include "datablocks/datatable.h"
#include "datablocks/itemdatamodel.h"
#include "datablocks/io/jsonconversionsutils.h"

namespace StereoVisionApp {

const char* Trajectory::OptDataTimeHeader = "Time";
const char* Trajectory::OptDataPosXHeader = "Trajectory X coordinate";
const char* Trajectory::OptDataPosYHeader = "Trajectory Y coordinate";
const char* Trajectory::OptDataPosZHeader = "Trajectory Z coordinate";
const char* Trajectory::OptDataRotXHeader = "Trajectory X orientation";
const char* Trajectory::OptDataRotYHeader = "Trajectory Y orientation";
const char* Trajectory::OptDataRotZHeader = "Trajectory Z orientation";

Trajectory::Trajectory(Project* parent) :
    DataBlock(parent),
    _optimizedTrajectoryData(nullptr)
{
    _plateformDefinition.boresight.setConstant(0);
    _plateformDefinition.leverArm.setConstant(0);

    _accelerometerMounting.R = Eigen::Matrix3d::Identity();
    _accelerometerMounting.t.setZero();

    _gyroMounting.R = Eigen::Matrix3d::Identity();
    _gyroMounting.t.setZero();

    _accelerometerId = 1;
    _gyroId = 1;

    extendDataModel();
}

StatusOptionalReturn<std::vector<Trajectory::TimeCartesianBlock>> Trajectory::loadAngularSpeedRawData() const {

    using RType = StatusOptionalReturn<std::vector<Trajectory::TimeCartesianBlock>>;

    QFile angspdFile(_angularSpeedFile);

    if (!angspdFile.open(QIODevice::ReadOnly)) {
        return RType::error("Could not open the angular speed file!");
    }

    QString lineData;

    std::vector<Trajectory::TimeCartesianBlock> ret;

    int lineCount = 0;

    QString sepPattern = "";

    for (int i = 0; i < _separators.size(); i++) {
        sepPattern += QRegularExpression::escape(_separators[i]);
        if (i != _separators.size()-1) {
            sepPattern += "|";
        }
    }

    QRegularExpression sep(sepPattern);

    //compute the minimum numbers of items in a line
    int minLineElements = _angularSpeedDefinition.w_t_col;

    if (_angularSpeedDefinition.w_x_col > minLineElements) {
        minLineElements = _angularSpeedDefinition.w_x_col;
    }
    if (_angularSpeedDefinition.w_y_col > minLineElements) {
        minLineElements = _angularSpeedDefinition.w_y_col;
    }
    if (_angularSpeedDefinition.w_z_col > minLineElements) {
        minLineElements = _angularSpeedDefinition.w_z_col;
    }
    if (_angularSpeedDefinition.w_w_col > minLineElements and _angularSpeedDefinition.angle_representation == Quaternion) {
        minLineElements = _angularSpeedDefinition.w_w_col;
    }

    minLineElements += 1;

    while (!angspdFile.atEnd()) {

        QByteArray line = angspdFile.readLine();
        QString str = QString::fromLocal8Bit(line);

        if (str.startsWith(_commentPattern)) {
            continue;
        }

        QStringList splitted = str.split(sep, Qt::SkipEmptyParts);

        if (splitted.size() < minLineElements) {
            continue;
        }

        Trajectory::TimeCartesianBlock entry;

        bool ok = true;

        entry.time = splitted[_angularSpeedDefinition.w_t_col].toDouble(&ok);
        entry.time = _angularSpeedDefinition.timeScale*entry.time + _angularSpeedDefinition.timeDelta;

        if (!ok) {
            continue;
        }

        double x;
        double y;
        double z;

        x = splitted[_angularSpeedDefinition.w_x_col].toDouble(&ok);

        if (!ok) {
            continue;
        }

        x *= _angularSpeedDefinition.sign_x;

        y = splitted[_angularSpeedDefinition.w_y_col].toDouble(&ok);

        if (!ok) {
            continue;
        }

        y *= _angularSpeedDefinition.sign_y;

        z = splitted[_angularSpeedDefinition.w_z_col].toDouble(&ok);

        if (!ok) {
            continue;
        }

        z *= _angularSpeedDefinition.sign_z;

        if (_angularSpeedDefinition.angle_representation == Quaternion) {

            double w = splitted[_angularSpeedDefinition.w_w_col].toDouble(&ok);

            if (!ok) {
                continue;
            }

            w *= _angularSpeedDefinition.sign_w;

            Eigen::Quaterniond quat(w, x, y, z);
            Eigen::Vector3d axis = Eigen::AngleAxisd(quat).axis();

            entry.val.x() = axis.x();
            entry.val.y() = axis.y();
            entry.val.z() = axis.z();

        } else if (_angularSpeedDefinition.angle_representation == EulerXYZ or
                _angularSpeedDefinition.angle_representation == EulerZYX) {

            double x_rad = x;
            double y_rad = y;
            double z_rad = z;

            if (_angularSpeedDefinition.angleUnit == Degree) {
                x_rad *= M_PI/180;
                y_rad *= M_PI/180;
                z_rad *= M_PI/180;
            }

            if (_angularSpeedDefinition.angleUnit == Gon) {
                x_rad *= M_PI/200;
                y_rad *= M_PI/200;
                z_rad *= M_PI/200;
            }

            Eigen::Vector3d axis;

            if (_angularSpeedDefinition.angle_representation == EulerXYZ) {
                Eigen::Quaterniond combinedRot =
                        Eigen::AngleAxisd(z_rad, Eigen::Vector3d::UnitZ())
                        * Eigen::AngleAxisd(y_rad,  Eigen::Vector3d::UnitY())
                        * Eigen::AngleAxisd(x_rad, Eigen::Vector3d::UnitX());

                Eigen::AngleAxisd axisAngle(combinedRot);

                double angle = axisAngle.angle();
                axis = axisAngle.axis();
                axis *= angle;
            }

            if (_angularSpeedDefinition.angle_representation == EulerZYX) {
                Eigen::Quaterniond combinedRot =
                        Eigen::AngleAxisd(x_rad, Eigen::Vector3d::UnitX())
                        * Eigen::AngleAxisd(y_rad,  Eigen::Vector3d::UnitY())
                        * Eigen::AngleAxisd(z_rad, Eigen::Vector3d::UnitZ());

                Eigen::AngleAxisd axisAngle(combinedRot);

                double angle = axisAngle.angle();
                axis = axisAngle.axis();
                axis *= angle;
            }

            entry.val.x() = axis.x();
            entry.val.y() = axis.y();
            entry.val.z() = axis.z();

        } else {

            entry.val.x() = x;
            entry.val.y() = y;
            entry.val.z() = z;

        }

        ret.push_back(entry);

        lineCount++;

    }

    return ret;

}

StatusOptionalReturn<std::vector<Trajectory::TimeCartesianBlock>> Trajectory::loadAccelerationRawData() const {

    using RType = StatusOptionalReturn<std::vector<Trajectory::TimeCartesianBlock>>;

    QFile accFile(_accelerationFile);

    if (!accFile.open(QIODevice::ReadOnly)) {
        return RType::error("Could not open accelerometer file!");
    }

    QString lineData;

    std::vector<Trajectory::TimeCartesianBlock> ret;

    int lineCount = 0;

    QString sepPattern = "";

    for (int i = 0; i < _separators.size(); i++) {
        sepPattern += QRegularExpression::escape(_separators[i]);
        if (i != _separators.size()-1) {
            sepPattern += "|";
        }
    }

    QRegularExpression sep(sepPattern);

    //compute the minimum numbers of items in a line
    int minLineElements = _accelerationDefinition.acc_t_col;

    if (_accelerationDefinition.acc_x_col > minLineElements) {
        minLineElements = _accelerationDefinition.acc_x_col;
    }
    if (_accelerationDefinition.acc_y_col > minLineElements) {
        minLineElements = _accelerationDefinition.acc_y_col;
    }
    if (_accelerationDefinition.acc_z_col > minLineElements) {
        minLineElements = _accelerationDefinition.acc_z_col;
    }

    minLineElements += 1;

    while (!accFile.atEnd()) {

        QByteArray line = accFile.readLine();
        QString str = QString::fromLocal8Bit(line);

        if (str.startsWith(_commentPattern)) {
            continue;
        }

        QStringList splitted = str.split(sep, Qt::SkipEmptyParts);

        if (splitted.size() < minLineElements) {
            continue;
        }

        Trajectory::TimeCartesianBlock entry;

        bool ok = true;

        entry.time = splitted[_accelerationDefinition.acc_t_col].toDouble(&ok);
        entry.time = _accelerationDefinition.timeScale*entry.time + _accelerationDefinition.timeDelta;

        if (!ok) {
            continue;
        }

        entry.val.x() = splitted[_accelerationDefinition.acc_x_col].toDouble(&ok);

        if (!ok) {
            continue;
        }

        entry.val.y() = splitted[_accelerationDefinition.acc_y_col].toDouble(&ok);

        if (!ok) {
            continue;
        }

        entry.val.z() = splitted[_accelerationDefinition.acc_z_col].toDouble(&ok);

        if (!ok) {
            continue;
        }

        ret.push_back(entry);

        lineCount++;

    }

    return ret;

}

StatusOptionalReturn<std::vector<Trajectory::TimeCartesianBlock>> Trajectory::loadPositionData() const {

    using RType = StatusOptionalReturn<std::vector<Trajectory::TimeCartesianBlock>>;

    RType retOpt = loadPositionRawData();

    if (!retOpt.isValid()) {
        return RType::error(retOpt.errorMessage());
    }

    std::vector<Trajectory::TimeCartesianBlock>& ret = retOpt.value();

    if (ret.empty()) {
        return RType::error("Empty trajectory positions!");
    }

    if (_positionDefinition.crs_epsg.isEmpty()) {
        return ret; // no transformation required
    }

    const char* wgs84_ecef = "EPSG:4978";

    PJ_CONTEXT* ctx = proj_context_create();

    PJ* toEcef = proj_create_crs_to_crs(ctx,
                                        _positionDefinition.crs_epsg.toStdString().c_str(),
                                        wgs84_ecef,
                                        nullptr);

    if (toEcef == 0) { //in case of error
        proj_context_destroy(ctx);
        return RType::error("Error when initilizing the proj transform from the trajectory to ECEF");
    }

    int delta_x = 1;
    int delta_y = 1;
    int delta_z = 1;

    if (ret.size() > 1) {

        std::byte* addr_x1 = reinterpret_cast<std::byte*>(&ret[0].val.x());
        std::byte* addr_x2 = reinterpret_cast<std::byte*>(&ret[1].val.x());

        std::byte* addr_y1 = reinterpret_cast<std::byte*>(&ret[0].val.y());
        std::byte* addr_y2 = reinterpret_cast<std::byte*>(&ret[1].val.y());

        std::byte* addr_z1 = reinterpret_cast<std::byte*>(&ret[0].val.z());
        std::byte* addr_z2 = reinterpret_cast<std::byte*>(&ret[1].val.z());

        delta_x = addr_x2 - addr_x1; //assumes elements in the vector are evenly spaced
        delta_y = addr_y2 - addr_y1;
        delta_z = addr_z2 - addr_z1;
    }

    proj_trans_generic(toEcef, PJ_FWD,
                       &ret[0].val.x(), delta_x, ret.size(),
                        &ret[0].val.y(), delta_y, ret.size(),
                        &ret[0].val.z(), delta_z, ret.size(),
                        nullptr, 0, 0);

    proj_destroy(toEcef);
    proj_context_destroy(ctx);

    return ret;

}


StatusOptionalReturn<Trajectory::TimeCartesianSequence> Trajectory::loadAngularSpeedSequence() const {

    using RType = StatusOptionalReturn<Trajectory::TimeCartesianSequence>;

    StatusOptionalReturn<std::vector<TimeCartesianBlock>> dataOpt = loadAngularSpeedRawData();

    if (!dataOpt.isValid()) {
        return RType::error(dataOpt.errorMessage());
    }

    std::vector<TimeCartesianBlock>& data = dataOpt.value();

    if (data.empty()) {
        return RType::error("Empty trajectory!");
    }

    for (TimeCartesianBlock & block : data) {
        block.val = _gyroMounting*block.val;
    }

    return IndexedTimeSequence<Eigen::Vector3d, double>(std::move(data));

}

StatusOptionalReturn<Trajectory::TimeCartesianSequence> Trajectory::loadAccelerationSequence() const {

    using RType = StatusOptionalReturn<Trajectory::TimeCartesianSequence>;

    StatusOptionalReturn<std::vector<TimeCartesianBlock>> dataOpt = loadAccelerationRawData();

    if (!dataOpt.isValid()) {
        return RType::error(dataOpt.errorMessage());
    }

    std::vector<TimeCartesianBlock>& data = dataOpt.value();

    if (data.empty()) {
        return RType::error("Empty trajectory!");
    }

    for (TimeCartesianBlock & block : data) {
        block.val = _accelerometerMounting*block.val;
    }

    return IndexedTimeSequence<Eigen::Vector3d, double>(std::move(data));

}

StatusOptionalReturn<Trajectory::TimeCartesianSequence> Trajectory::loadOrientationSequence() const {

    using RType = StatusOptionalReturn<Trajectory::TimeCartesianSequence>;

    StatusOptionalReturn<std::vector<TimeCartesianBlock>> dataOpt = loadOrientationRawData();

    if (!dataOpt.isValid()) {
        return RType::error(dataOpt.errorMessage());
    }

    std::vector<TimeCartesianBlock>& data = dataOpt.value();

    if (data.empty()) {
        return RType::error("Empty trajectory!");
    }

    return IndexedTimeSequence<Eigen::Vector3d, double>(std::move(data));

}

StatusOptionalReturn<Trajectory::TimeCartesianSequence> Trajectory::loadPositionSequence() const {

    using RType = StatusOptionalReturn<Trajectory::TimeCartesianSequence>;

    StatusOptionalReturn<std::vector<TimeCartesianBlock>> dataOpt = loadPositionData();

    if (!dataOpt.isValid()) {
        return RType::error(dataOpt.errorMessage());
    }

    std::vector<TimeCartesianBlock>& data = dataOpt.value();

    if (data.empty()) {
        return RType::error("Empty trajectory!");
    }

    return IndexedTimeSequence<Eigen::Vector3d, double>(std::move(data));
}

StatusOptionalReturn<std::vector<Eigen::Vector3f> > Trajectory::loadTrajectoryPathInProjectLocalFrame() const {

    using RType = StatusOptionalReturn<std::vector<Eigen::Vector3f> >;

    std::optional<StereoVision::Geometry::AffineTransform<float>> transform2projFrame = std::nullopt;

    Project* proj = getProject();

    if (proj != nullptr) {
        if (proj->hasLocalCoordinateFrame()) {
            transform2projFrame = proj->ecef2local();
        }
    }

    StatusOptionalReturn<std::vector<TimeCartesianBlock>> dataOpt = loadPositionData();

    if (!dataOpt.isValid()) {
        return RType::error(dataOpt.errorMessage());
    }

    std::vector<TimeCartesianBlock>& data = dataOpt.value();

    std::vector<Eigen::Vector3f> path(data.size());


    if (transform2projFrame.has_value()) {

        for (int i = 0; i < data.size(); i++) {

            Eigen::Vector3f point = data[i].val.cast<float>();
            point = transform2projFrame.value() * point;

            path[i] = point;
        }

    } else {

        for (int i = 0; i < data.size(); i++) {

            Eigen::Vector3f point = data[i].val.cast<float>();

            path[i] = point;
        }
    }

    return path;

}
StatusOptionalReturn<std::vector<StereoVision::Geometry::AffineTransform<float> > > Trajectory::loadTrajectoryInProjectLocalFrame(bool optimized) const {

    using RType = StatusOptionalReturn<std::vector<StereoVision::Geometry::AffineTransform<float> > >;

    if (optimized) {

        if (_optimizedTrajectoryData == nullptr) {
            return RType::error("Optimized trajectory uninitialized!");
        }

        int nRows = _optimizedTrajectoryData->nRows();

        std::vector<StereoVision::Geometry::AffineTransform<float>> data;
        data.reserve(nRows);

        for (int i = 0; i < nRows; i++) {

            StereoVision::Geometry::AffineTransform<float> transform;

            transform.t.x() = _optimizedTrajectoryData->getData(OptDataPosXHeader, i).toDouble();
            transform.t.y() = _optimizedTrajectoryData->getData(OptDataPosYHeader, i).toDouble();
            transform.t.z() = _optimizedTrajectoryData->getData(OptDataPosZHeader, i).toDouble();

            Eigen::Vector3f r;

            r.x() = _optimizedTrajectoryData->getData(OptDataRotXHeader, i).toDouble();
            r.y() = _optimizedTrajectoryData->getData(OptDataRotYHeader, i).toDouble();
            r.z() = _optimizedTrajectoryData->getData(OptDataRotZHeader, i).toDouble();

            transform.R = StereoVision::Geometry::rodriguezFormula(r);

            data.push_back(transform);

        }

        return data;
    }

    std::optional<StereoVision::Geometry::AffineTransform<float>> transform2projFrame = std::nullopt;

    Project* proj = getProject();

    if (proj != nullptr) {
        if (proj->hasLocalCoordinateFrame()) {
            transform2projFrame = proj->ecef2local();
        }
    }

    StatusOptionalReturn<std::vector<TimeTrajectoryBlock>> dataOpt = loadTrajectoryData();

    if (!dataOpt.isValid()) {
        return RType::error(dataOpt.errorMessage());
    }

    std::vector<TimeTrajectoryBlock>& data = dataOpt.value();

    std::vector<StereoVision::Geometry::AffineTransform<float>> path(data.size());


    if (transform2projFrame.has_value()) {

        for (int i = 0; i < data.size(); i++) {

            StereoVision::Geometry::AffineTransform<float> pose = data[i].val.cast<float>().toAffineTransform();
            pose = transform2projFrame.value() * pose;

            path[i] = pose;
        }

    } else {

        for (int i = 0; i < data.size(); i++) {

            StereoVision::Geometry::AffineTransform<float> pose = data[i].val.cast<float>().toAffineTransform();

            path[i] = pose;
        }
    }

    return path;

}

StatusOptionalReturn<std::vector<Trajectory::TimeCartesianBlock>> Trajectory::loadPositionRawData() const {

    using RType = StatusOptionalReturn<std::vector<Trajectory::TimeCartesianBlock>>;

    QFile trajFile(_positionFile);

    if (!trajFile.open(QIODevice::ReadOnly)) {
        return RType::error("Could not open position file!");
    }

    QString lineData;

    std::vector<Trajectory::TimeCartesianBlock> ret;

    int lineCount = 0;

    QString sepPattern = "";

    for (int i = 0; i < _separators.size(); i++) {
        sepPattern += QRegularExpression::escape(_separators[i]);
        if (i != _separators.size()-1) {
            sepPattern += "|";
        }
    }

    QRegularExpression sep(sepPattern);

    //compute the minimum numbers of items in a line
    int minLineElements = _positionDefinition.pos_t_col;

    if (_positionDefinition.pos_x_col > minLineElements) {
        minLineElements = _positionDefinition.pos_x_col;
    }
    if (_positionDefinition.pos_y_col > minLineElements) {
        minLineElements = _positionDefinition.pos_y_col;
    }
    if (_positionDefinition.pos_z_col > minLineElements) {
        minLineElements = _positionDefinition.pos_z_col;
    }

    minLineElements += 1;

    while (!trajFile.atEnd()) {

        QByteArray line = trajFile.readLine();
        QString str = QString::fromLocal8Bit(line);

        if (str.startsWith(_commentPattern)) {
            continue;
        }

        QStringList splitted = str.split(sep, Qt::SkipEmptyParts);

        if (splitted.size() < minLineElements) {
            continue;
        }

        Trajectory::TimeCartesianBlock entry;

        bool ok = true;

        entry.time = splitted[_positionDefinition.pos_t_col].toDouble(&ok);
        entry.time = _positionDefinition.timeScale*entry.time + _positionDefinition.timeDelta;

        if (!ok) {
            continue;
        }

        entry.val.x() = splitted[_positionDefinition.pos_x_col].toDouble(&ok);

        if (!ok) {
            continue;
        }

        entry.val.y() = splitted[_positionDefinition.pos_y_col].toDouble(&ok);

        if (!ok) {
            continue;
        }

        entry.val.z() = splitted[_positionDefinition.pos_z_col].toDouble(&ok);

        if (!ok) {
            continue;
        }

        ret.push_back(entry);

        lineCount++;

    }

    return ret;
}

/*!
 * \brief loadPositionRawData load the orientation raw data (i.e. without any geodetic conversion).
 * \return a vector of time orientation blocks (represented as axis angles)
 */
StatusOptionalReturn<std::vector<Trajectory::TimeCartesianBlock>> Trajectory::loadOrientationRawData() const {

    using RType = StatusOptionalReturn<std::vector<Trajectory::TimeCartesianBlock>>;

    QFile trajFile(_orientationFile);

    if (!trajFile.open(QIODevice::ReadOnly)) {
        return RType::error("Could not open orientation file!");
    }

    QString lineData;

    std::vector<Trajectory::TimeCartesianBlock> ret;

    int lineCount = 0;

    QString sepPattern = "";

    for (int i = 0; i < _separators.size(); i++) {
        sepPattern += QRegularExpression::escape(_separators[i]);
        if (i != _separators.size()-1) {
            sepPattern += "|";
        }
    }

    QRegularExpression sep(sepPattern);

    //compute the minimum numbers of items in a line
    int minLineElements = _orientationDefinition.orient_t_col;

    if (_orientationDefinition.orient_x_col > minLineElements) {
        minLineElements = _orientationDefinition.orient_x_col;
    }
    if (_orientationDefinition.orient_y_col > minLineElements) {
        minLineElements = _orientationDefinition.orient_y_col;
    }
    if (_orientationDefinition.orient_z_col > minLineElements) {
        minLineElements = _orientationDefinition.orient_z_col;
    }
    if (_orientationDefinition.orient_w_col > minLineElements and _orientationDefinition.angle_representation == Quaternion) {
        minLineElements = _orientationDefinition.orient_w_col;
    }

    minLineElements += 1;

    while (!trajFile.atEnd()) {

        QByteArray line = trajFile.readLine();
        QString str = QString::fromLocal8Bit(line);

        if (str.startsWith(_commentPattern)) {
            continue;
        }

        QStringList splitted = str.split(sep, Qt::SkipEmptyParts);

        if (splitted.size() < minLineElements) {
            continue;
        }

        Trajectory::TimeCartesianBlock entry;

        bool ok = true;

        entry.time = splitted[_orientationDefinition.orient_t_col].toDouble(&ok);
        entry.time = _orientationDefinition.timeScale*entry.time + _orientationDefinition.timeDelta;

        if (!ok) {
            continue;
        }

        double x;
        double y;
        double z;

        x = splitted[_orientationDefinition.orient_x_col].toDouble(&ok);

        if (!ok) {
            continue;
        }

        x *= _orientationDefinition.sign_x;

        y = splitted[_orientationDefinition.orient_y_col].toDouble(&ok);

        if (!ok) {
            continue;
        }

        y *= _orientationDefinition.sign_y;

        z = splitted[_orientationDefinition.orient_z_col].toDouble(&ok);

        if (!ok) {
            continue;
        }

        z *= _orientationDefinition.sign_z;

        if (_orientationDefinition.angle_representation == Quaternion) {

            double w = splitted[_orientationDefinition.orient_w_col].toDouble(&ok);

            if (!ok) {
                continue;
            }

            w *= _orientationDefinition.sign_w;

            Eigen::Quaterniond quat(w, x, y, z);
            Eigen::Vector3d axis = Eigen::AngleAxisd(quat).axis();

            entry.val.x() = axis.x();
            entry.val.y() = axis.y();
            entry.val.z() = axis.z();

        } else if (_orientationDefinition.angle_representation == EulerXYZ or
                _orientationDefinition.angle_representation == EulerZYX) {

            double x_rad = x;
            double y_rad = y;
            double z_rad = z;

            if (_orientationDefinition.angleUnit == Degree) {
                x_rad *= M_PI/180;
                y_rad *= M_PI/180;
                z_rad *= M_PI/180;
            }

            if (_orientationDefinition.angleUnit == Gon) {
                x_rad *= M_PI/200;
                y_rad *= M_PI/200;
                z_rad *= M_PI/200;
            }

            Eigen::Vector3d axis;

            if (_orientationDefinition.angle_representation == EulerXYZ) {
                Eigen::Quaterniond combinedRot =
                        Eigen::AngleAxisd(z_rad, Eigen::Vector3d::UnitZ())
                        * Eigen::AngleAxisd(y_rad,  Eigen::Vector3d::UnitY())
                        * Eigen::AngleAxisd(x_rad, Eigen::Vector3d::UnitX());

                Eigen::AngleAxisd axisAngle(combinedRot);

                double angle = axisAngle.angle();
                axis = axisAngle.axis();
                axis *= angle;
            }

            if (_orientationDefinition.angle_representation == EulerZYX) {
                Eigen::Quaterniond combinedRot =
                        Eigen::AngleAxisd(x_rad, Eigen::Vector3d::UnitX())
                        * Eigen::AngleAxisd(y_rad,  Eigen::Vector3d::UnitY())
                        * Eigen::AngleAxisd(z_rad, Eigen::Vector3d::UnitZ());

                Eigen::AngleAxisd axisAngle(combinedRot);

                double angle = axisAngle.angle();
                axis = axisAngle.axis();
                axis *= angle;
            }

            entry.val.x() = axis.x();
            entry.val.y() = axis.y();
            entry.val.z() = axis.z();

        } else {

            entry.val.x() = x;
            entry.val.y() = y;
            entry.val.z() = z;

        }

        ret.push_back(entry);

        lineCount++;

    }

    return ret;

}

StatusOptionalReturn<std::vector<Trajectory::TimeTrajectoryBlock>> Trajectory::loadTrajectoryData() const {

    using RType = StatusOptionalReturn<std::vector<Trajectory::TimeTrajectoryBlock>>;

    //TODO: generalize this function with interpolation
    StatusOptionalReturn<std::vector<TimeCartesianBlock>> posECEFOpt = loadPositionData();
    StatusOptionalReturn<std::vector<TimeCartesianBlock>> orientationRawOpt = loadOrientationRawData();

    if (!posECEFOpt.isValid()) {
        return RType::error(posECEFOpt.errorMessage());
    }

    if (!orientationRawOpt.isValid()) {
        return RType::error(posECEFOpt.errorMessage());
    }

    std::vector<TimeCartesianBlock>& posECEF = posECEFOpt.value();
    std::vector<TimeCartesianBlock>& orientationRaw = orientationRawOpt.value();

    if (posECEF.size() != orientationRaw.size()) {
        return RType::error("Size mismatch between position and orientation data!");
    }

    Eigen::Array<double,3,Eigen::Dynamic> posData;
    posData.resize(3,posECEF.size());

    for (int i = 0; i < posECEF.size(); i++) {
        posData(0,i) = posECEF[i].val.x();
        posData(1,i) = posECEF[i].val.y();
        posData(2,i) = posECEF[i].val.z();
    }

    std::vector<Trajectory::TimeTrajectoryBlock> trajectory(posECEF.size());

    if (!geoReferenceSupportActive()) {


        for (int i = 0; i < posECEF.size(); i++) {

            trajectory[i].time = posECEF[i].time;
            trajectory[i].val = PoseType(orientationRaw[i].val, posECEF[i].val);

        }

        return trajectory;
    }

    const std::string wgs84ECEF = "EPSG:4978";

    Geo::TopocentricConvention topoConv = static_cast<Geo::TopocentricConvention>(_orientationDefinition.topocentric_convention);
    auto optLocalFrames = Geo::getLTPC2ECEF(posData, wgs84ECEF, topoConv);

    if (!optLocalFrames.has_value()) {
        return RType::error("Error when evaluating trajectory local frame!");
    }

    std::vector<StereoVision::Geometry::AffineTransform<double>>& localFrames = optLocalFrames.value();

    if (localFrames.size() != posECEF.size()) {
        return RType::error("Mismatch in the number of local frames computed!");
    }

    PoseType plateform2sensor(_plateformDefinition.boresight, _plateformDefinition.leverArm);

    for (int i = 0; i < posECEF.size(); i++) {

        Eigen::Matrix3d body2local = StereoVision::Geometry::rodriguezFormula(orientationRaw[i].val);
        StereoVision::Geometry::AffineTransform<double>& local2ecef = localFrames[i];

        trajectory[i].time = posECEF[i].time;
        StereoVision::Geometry::AffineTransform<double> transformed(local2ecef.R*body2local, local2ecef.t);
        trajectory[i].val = PoseType(StereoVision::Geometry::inverseRodriguezFormula(transformed.R), transformed.t) * plateform2sensor;

    }

    return trajectory;
}

StatusOptionalReturn<Trajectory::TimeTrajectorySequence> Trajectory::loadTrajectorySequence() const {

    using RType = StatusOptionalReturn<Trajectory::TimeTrajectorySequence>;

    if (_trajectoryCache.has_value()) {
        return _trajectoryCache.value();
    }

    StatusOptionalReturn<std::vector<TimeTrajectoryBlock>> dataOpt = loadTrajectoryData();

    if (!dataOpt.isValid()) {
        return RType::error(dataOpt.errorMessage());
    }

    std::vector<TimeTrajectoryBlock>& data = dataOpt.value();

    if (data.empty()) {
        return RType::error("Empty trajectory!");
    }

    _trajectoryCache = TimeTrajectorySequence(std::move(data));

    return _trajectoryCache.value();
}

StatusOptionalReturn<Trajectory::TimeTrajectorySequence> Trajectory::loadTrajectoryProjectLocalFrameSequence() const {

    using RType = StatusOptionalReturn<Trajectory::TimeTrajectorySequence>;

    std::optional<StereoVision::Geometry::AffineTransform<float>> transform2projFrame = std::nullopt;

    Project* proj = getProject();

    if (proj != nullptr) {
        if (proj->hasLocalCoordinateFrame()) {
            transform2projFrame = proj->ecef2local();
        }
    }

    StatusOptionalReturn<std::vector<TimeTrajectoryBlock>> dataOpt = loadTrajectoryData();

    if (!dataOpt.isValid()) {
        return RType::error(dataOpt.errorMessage());
    }

    std::vector<TimeTrajectoryBlock>& data = dataOpt.value();

    if (data.empty()) {
        return RType::error("Empty trajectory!");
    }


    if (transform2projFrame.has_value()) {

        StereoVision::Geometry::AffineTransform<double> transform = transform2projFrame.value().cast<double>();

        for (int i = 0; i < data.size(); i++) {

            PoseType pose = data[i].val; //sensor 2 ecef
            // plateform 2 project local

            Eigen::Vector3d pos = transform*pose.t;

            Eigen::Vector3d rot = pose.r;
            Eigen::Matrix3d R = transform.R*StereoVision::Geometry::rodriguezFormula<double>(rot);
            rot = StereoVision::Geometry::inverseRodriguezFormula<double>(R);

            data[i].val = PoseType(rot, pos);
        }

    }

    return TimeTrajectorySequence(std::move(data));

}

StatusOptionalReturn<Trajectory::TimeTrajectorySequence> Trajectory::optimizedTrajectory() const {

    using RType = StatusOptionalReturn<Trajectory::TimeTrajectorySequence>;

    if (_optimizedTrajectoryData == nullptr) {
        return RType::error("Uninitilized optimized trajectory");
    }

    int nRows = _optimizedTrajectoryData->nRows();

    std::vector<Trajectory::TimeTrajectorySequence::TimedElement> data;
    data.reserve(nRows);

    for (int i = 0; i < nRows; i++) {

        Trajectory::TimeTrajectorySequence::TimedElement elem;

        elem.time = _optimizedTrajectoryData->getData(OptDataTimeHeader, i).toDouble();

        elem.val.t.x() = _optimizedTrajectoryData->getData(OptDataPosXHeader, i).toDouble();
        elem.val.t.y() = _optimizedTrajectoryData->getData(OptDataPosYHeader, i).toDouble();
        elem.val.t.z() = _optimizedTrajectoryData->getData(OptDataPosZHeader, i).toDouble();

        elem.val.r.x() = _optimizedTrajectoryData->getData(OptDataRotXHeader, i).toDouble();
        elem.val.r.y() = _optimizedTrajectoryData->getData(OptDataRotYHeader, i).toDouble();
        elem.val.r.z() = _optimizedTrajectoryData->getData(OptDataRotZHeader, i).toDouble();

        data.push_back(elem);

    }

    return Trajectory::TimeTrajectorySequence(data);
}

bool Trajectory::geoReferenceSupportActive() const  {
    return !_positionDefinition.crs_epsg.isEmpty();
}

Eigen::Array<float,3, Eigen::Dynamic> Trajectory::getLocalPointsEcef() const  {

    StatusOptionalReturn<std::vector<TimeCartesianBlock>> positionDataOpt = loadPositionData();

    Eigen::Array<float,3, Eigen::Dynamic> ret;

    if (!positionDataOpt.isValid()) {
        return ret;
    }

    std::vector<TimeCartesianBlock>& positionData = positionDataOpt.value();

    if (positionData.size() == 0) {
        return ret;
    }

    ret.resize(3,1);
    ret(0,0) = positionData[0].val.x();
    ret(1,0) = positionData[0].val.y();
    ret(2,0) = positionData[0].val.z();

    return ret;
}

QString Trajectory::getCoordinateReferenceSystemDescr(int role) const {

    if (role == DefaultCRSRole) {
        return _positionDefinition.crs_epsg;
    }
    return _positionDefinition.crs_epsg;
}

void Trajectory::setPositionFile(QString const& path) {

    if (path != _positionFile) {
        _positionFile = path;
        Q_EMIT trajectoryDataChanged();
    }
}

void Trajectory::setPositionEpsg(QString const& epsg) {

    if (epsg != _positionDefinition.crs_epsg) {
        _positionDefinition.crs_epsg = epsg;
        Q_EMIT trajectoryDataChanged();
    }
}

void Trajectory::setPositionTimeScale(double scale) {
    if (_positionDefinition.timeScale != scale) {
        _positionDefinition.timeScale = scale;
        Q_EMIT trajectoryDataChanged();
    }
}

void Trajectory::setPositionTimeDelta(double delta) {
    if (_positionDefinition.timeDelta != delta) {
        _positionDefinition.timeDelta = delta;
        Q_EMIT trajectoryDataChanged();
    }
}

void Trajectory::setPositionColumn(Axis axis, int col) {
    switch (axis) {
    case T:
        if (col != _positionDefinition.pos_t_col){
            _positionDefinition.pos_t_col = col;
            Q_EMIT trajectoryDataChanged();
        }
        break;
    case X:
        if (col != _positionDefinition.pos_x_col) {
            _positionDefinition.pos_x_col = col;
            Q_EMIT trajectoryDataChanged();
        }
        break;
    case Y:
        if (col != _positionDefinition.pos_y_col) {
            _positionDefinition.pos_y_col = col;
            Q_EMIT trajectoryDataChanged();
        }
        break;
    case Z:
        if (col != _positionDefinition.pos_z_col) {
            _positionDefinition.pos_z_col = col;
            Q_EMIT trajectoryDataChanged();
        }
        break;
    default:
        break;
    }
}
void Trajectory::setAccelerometerFile(QString const& path) {
    if (path != _accelerationFile) {
        _accelerationFile = path;
        Q_EMIT trajectoryDataChanged();
    }
}

void Trajectory::setAccelerometerTimeScale(double scale) {
    if (_accelerationDefinition.timeScale != scale) {
        _accelerationDefinition.timeScale = scale;
        Q_EMIT trajectoryDataChanged();
    }
}

void Trajectory::setAccelerometerTimeDelta(double delta) {
    if (_accelerationDefinition.timeDelta != delta) {
        _accelerationDefinition.timeDelta = delta;
        Q_EMIT trajectoryDataChanged();
    }
}

void Trajectory::setAccelerometerColumn(Axis axis, int col) {
    switch (axis) {
    case T:
        if (col != _accelerationDefinition.acc_t_col){
            _accelerationDefinition.acc_t_col = col;
            Q_EMIT trajectoryDataChanged();
        }
        break;
    case X:
        if (col != _accelerationDefinition.acc_x_col) {
            _accelerationDefinition.acc_x_col = col;
            Q_EMIT trajectoryDataChanged();
        }
        break;
    case Y:
        if (col != _accelerationDefinition.acc_y_col) {
            _accelerationDefinition.acc_y_col = col;
            Q_EMIT trajectoryDataChanged();
        }
        break;
    case Z:
        if (col != _accelerationDefinition.acc_z_col) {
            _accelerationDefinition.acc_z_col = col;
            Q_EMIT trajectoryDataChanged();
        }
        break;
    default:
        break;
    }

}

floatParameter Trajectory::optAccelerometerBiasX() const {
    return _accelerometerParameters._o_biasX;
}
floatParameter Trajectory::optAccelerometerBiasY() const {
    return _accelerometerParameters._o_biasY;
}
floatParameter Trajectory::optAccelerometerBiasZ() const {
    return _accelerometerParameters._o_biasZ;
}

void Trajectory::setOptAccelerometerBiasX(floatParameter const& biasX) {

    if (!_accelerometerParameters._o_biasX.isApproximatlyEqual(biasX)) {
        _accelerometerParameters._o_biasX = biasX;
        Q_EMIT accelerometerBiasXChanged();
    }

}
void Trajectory::setOptAccelerometerBiasY(floatParameter const& biasY) {

    if (!_accelerometerParameters._o_biasY.isApproximatlyEqual(biasY)) {
        _accelerometerParameters._o_biasY = biasY;
        Q_EMIT accelerometerBiasYChanged();
    }

}
void Trajectory::setOptAccelerometerBiasZ(floatParameter const& biasZ) {

    if (!_accelerometerParameters._o_biasZ.isApproximatlyEqual(biasZ)) {
        _accelerometerParameters._o_biasZ = biasZ;
        Q_EMIT accelerometerBiasZChanged();
    }

}

floatParameter Trajectory::optAccelerometerScale() const {
    return _accelerometerParameters._o_scale;
}

void Trajectory::setOptAccelerometerScale(floatParameter const& scale) {
    if (!_accelerometerParameters._o_scale.isApproximatlyEqual(scale)) {
        _accelerometerParameters._o_scale = scale;
        Q_EMIT accelerometerScaleChanged();
    }
}

void Trajectory::setOrientationFile(QString const& path) {
    _orientationFile = path;
}

void Trajectory::setOrientationTopocentricConvention(TopocentricConvention convention) {
    _orientationDefinition.topocentric_convention = convention;
}

void Trajectory::setOrientationAngleRepresentation(AngleRepresentation representation) {
    _orientationDefinition.angle_representation = representation;
}

void Trajectory::setOrientationAngleUnits(AngleUnits units) {
    _orientationDefinition.angleUnit = units;
}

void Trajectory::setOrientationTimeScale(double scale) {
    if (_orientationDefinition.timeScale != scale) {
        _orientationDefinition.timeScale = scale;
        Q_EMIT trajectoryDataChanged();
    }
}

void Trajectory::setOrientationTimeDelta(double delta){
    if (_orientationDefinition.timeDelta != delta) {
        _orientationDefinition.timeDelta = delta;
        Q_EMIT trajectoryDataChanged();
    }
}

void Trajectory::setOrientationColumn(Axis axis, int col) {
    switch (axis) {
    case T:
        _orientationDefinition.orient_t_col = col;
        break;
    case X:
        _orientationDefinition.orient_x_col = col;
        break;
    case Y:
        _orientationDefinition.orient_y_col = col;
        break;
    case Z:
        _orientationDefinition.orient_z_col = col;
        break;
    case W:
        _orientationDefinition.orient_w_col = col;
        break;
    }
}

void Trajectory::setOrientationSign(Axis axis, int sign) {
    switch (axis) {
    case T:
        return;
    case X:
        _orientationDefinition.sign_x = sign;
        break;
    case Y:
        _orientationDefinition.sign_y = sign;
        break;
    case Z:
        _orientationDefinition.sign_z = sign;
        break;
    case W:
        _orientationDefinition.sign_w = sign;
        break;
    }
}

void Trajectory::setGyroFile(QString const& path) {
    if (_angularSpeedFile != path) {
        _angularSpeedFile = path;
        Q_EMIT trajectoryDataChanged();
    }
}

void Trajectory::setGyroAngleRepresentation(AngleRepresentation representation) {
    _angularSpeedDefinition.angle_representation = representation;
}

void Trajectory::setGyroAngleUnits(AngleUnits units) {
    _angularSpeedDefinition.angleUnit = units;
}

void Trajectory::setGyroTimeScale(double scale) {
    if (_angularSpeedDefinition.timeScale != scale) {
        _angularSpeedDefinition.timeScale = scale;
        Q_EMIT trajectoryDataChanged();
    }
}

void Trajectory::setGyroTimeDelta(double delta) {
    if (_angularSpeedDefinition.timeDelta != delta) {
        _angularSpeedDefinition.timeDelta = delta;
        Q_EMIT trajectoryDataChanged();
    }
}

void Trajectory::setGyroColumn(Axis axis, int col) {
    switch (axis) {
    case T:
        _angularSpeedDefinition.w_t_col = col;
        break;
    case X:
        _angularSpeedDefinition.w_x_col = col;
        break;
    case Y:
        _angularSpeedDefinition.w_y_col = col;
        break;
    case Z:
        _angularSpeedDefinition.w_z_col = col;
        break;
    case W:
        _angularSpeedDefinition.w_w_col = col;
        break;
    }

}

void Trajectory::setGyroSign(Axis axis, int sign) {
    switch (axis) {
    case T:
        return;
    case X:
        _angularSpeedDefinition.sign_x = sign;
        break;
    case Y:
        _angularSpeedDefinition.sign_y = sign;
        break;
    case Z:
        _angularSpeedDefinition.sign_z = sign;
        break;
    case W:
        _angularSpeedDefinition.sign_w = sign;
        break;
    }

}

DataTable* Trajectory::getOptimizedDataTable() {

    if (_optimizedTrajectoryData == nullptr) {
        _optimizedTrajectoryData = new DataTable(this);

        connect(_optimizedTrajectoryData, &DataBlock::datablockChanged,
                this, &Trajectory::optimizedTrajectoryDataChanged);
    }

    return _optimizedTrajectoryData;

}
bool Trajectory::hasOptimizedTrajectory() const {
    if (_optimizedTrajectoryData == nullptr) {
        return false;
    }

    return _optimizedTrajectoryData->nRows() > 0;
}

QJsonObject Trajectory::encodeJson() const {

    QJsonObject obj;

    QJsonObject accelerationDefinition;

    accelerationDefinition.insert("time_delta", _accelerationDefinition.timeDelta);
    accelerationDefinition.insert("time_scale", _accelerationDefinition.timeScale);

    accelerationDefinition.insert("acc_t_col", _accelerationDefinition.acc_t_col);
    accelerationDefinition.insert("acc_x_col", _accelerationDefinition.acc_x_col);
    accelerationDefinition.insert("acc_y_col", _accelerationDefinition.acc_y_col);
    accelerationDefinition.insert("acc_z_col", _accelerationDefinition.acc_z_col);

    QJsonObject accMountingObj = affineTransform2json(_accelerometerMounting);

    accelerationDefinition.insert("mounting", accMountingObj);

    obj.insert("accelerationDefinition", accelerationDefinition);
    obj.insert("accelerationFile", _accelerationFile);

    QJsonObject angularSpeedDefinition;

    QMetaEnum e = QMetaEnum::fromType<AngleRepresentation>();
    angularSpeedDefinition.insert("angle_representation", e.valueToKey(_angularSpeedDefinition.angle_representation));
    e = QMetaEnum::fromType<AngleUnits>();
    angularSpeedDefinition.insert("angleUnit", e.valueToKey(_angularSpeedDefinition.angleUnit));

    angularSpeedDefinition.insert("time_delta", _angularSpeedDefinition.timeDelta);
    angularSpeedDefinition.insert("time_scale", _angularSpeedDefinition.timeScale);

    angularSpeedDefinition.insert("pos_t_col", _angularSpeedDefinition.w_t_col);
    angularSpeedDefinition.insert("pos_x_col", _angularSpeedDefinition.w_x_col);
    angularSpeedDefinition.insert("pos_y_col", _angularSpeedDefinition.w_y_col);
    angularSpeedDefinition.insert("pos_z_col", _angularSpeedDefinition.w_z_col);
    angularSpeedDefinition.insert("pos_w_col", _angularSpeedDefinition.w_w_col);

    angularSpeedDefinition.insert("sign_x", _angularSpeedDefinition.sign_x);
    angularSpeedDefinition.insert("sign_y", _angularSpeedDefinition.sign_y);
    angularSpeedDefinition.insert("sign_z", _angularSpeedDefinition.sign_z);
    angularSpeedDefinition.insert("sign_w", _angularSpeedDefinition.sign_w);

    QJsonObject gyroMountingObj = affineTransform2json(_gyroMounting);
    angularSpeedDefinition.insert("mounting", gyroMountingObj);

    obj.insert("angularSpeedDefinition", angularSpeedDefinition);
    obj.insert("angularSpeedFile", _angularSpeedFile);

    QJsonObject positionDefinition;

    positionDefinition.insert("crs_epsg", _positionDefinition.crs_epsg);

    positionDefinition.insert("time_delta", _positionDefinition.timeDelta);
    positionDefinition.insert("time_scale", _positionDefinition.timeScale);

    positionDefinition.insert("pos_t_col", _positionDefinition.pos_t_col);
    positionDefinition.insert("pos_x_col", _positionDefinition.pos_x_col);
    positionDefinition.insert("pos_y_col", _positionDefinition.pos_y_col);
    positionDefinition.insert("pos_z_col", _positionDefinition.pos_z_col);

    obj.insert("positionDefinition", positionDefinition);
    obj.insert("positionFile", _positionFile);

    QJsonObject orientationDefinition;

    e = QMetaEnum::fromType<TopocentricConvention>();
    orientationDefinition.insert("topocentric_convention", e.valueToKey(_orientationDefinition.topocentric_convention));
    e = QMetaEnum::fromType<AngleRepresentation>();
    orientationDefinition.insert("angle_representation", e.valueToKey(_orientationDefinition.angle_representation));
    e = QMetaEnum::fromType<AngleUnits>();
    orientationDefinition.insert("angleUnit", e.valueToKey(_orientationDefinition.angleUnit));

    orientationDefinition.insert("time_delta", _orientationDefinition.timeDelta);
    orientationDefinition.insert("time_scale", _orientationDefinition.timeScale);

    orientationDefinition.insert("pos_t_col", _orientationDefinition.orient_t_col);
    orientationDefinition.insert("pos_x_col", _orientationDefinition.orient_x_col);
    orientationDefinition.insert("pos_y_col", _orientationDefinition.orient_y_col);
    orientationDefinition.insert("pos_z_col", _orientationDefinition.orient_z_col);
    orientationDefinition.insert("pos_w_col", _orientationDefinition.orient_w_col);

    orientationDefinition.insert("sign_x", _orientationDefinition.sign_x);
    orientationDefinition.insert("sign_y", _orientationDefinition.sign_y);
    orientationDefinition.insert("sign_z", _orientationDefinition.sign_z);
    orientationDefinition.insert("sign_w", _orientationDefinition.sign_w);

    obj.insert("orientationDefinition", orientationDefinition);
    obj.insert("orientationFile", _orientationFile);

    obj.insert("accelerometerId", _accelerometerId);
    obj.insert("gyroId", _gyroId);

    if (_optimizedTrajectoryData != nullptr) {
        obj.insert("optimizedTrajectory", static_cast<DataBlock*>(_optimizedTrajectoryData)->encodeJson());
    }

    QJsonArray separatorsStrings;

    for (int i = 0; i < _separators.size(); i++) {
        separatorsStrings.push_back(QJsonValue(_separators[i]));
    }

    obj.insert("inputDataSeparators", separatorsStrings);

    obj.insert("commentPattern", _commentPattern);

    return obj;

}
void Trajectory::configureFromJson(QJsonObject const& data) {

    if (data.contains("accelerationDefinition")) {

        QJsonObject accelerationDefinition = data.value("accelerationDefinition").toObject();

        if (accelerationDefinition.contains("time_delta")) {
            _accelerationDefinition.timeDelta = accelerationDefinition.value("time_delta").toDouble(0);
        } else {
            _accelerationDefinition.timeDelta = 0;
        }

        if (accelerationDefinition.contains("time_scale")) {
            _accelerationDefinition.timeScale = accelerationDefinition.value("time_scale").toDouble(1);
        } else {
            _accelerationDefinition.timeScale = 1;
        }

        if (accelerationDefinition.contains("acc_t_col")) {
            _accelerationDefinition.acc_t_col = accelerationDefinition.value("acc_t_col").toInt(-1);
        }

        if (accelerationDefinition.contains("acc_x_col")) {
            _accelerationDefinition.acc_x_col = accelerationDefinition.value("acc_x_col").toInt(-1);
        }

        if (accelerationDefinition.contains("acc_y_col")) {
            _accelerationDefinition.acc_y_col = accelerationDefinition.value("acc_y_col").toInt(-1);
        }

        if (accelerationDefinition.contains("acc_z_col")) {
            _accelerationDefinition.acc_z_col = accelerationDefinition.value("acc_z_col").toInt(-1);
        }

        if (accelerationDefinition.contains("mounting")) {
            auto accMounting = json2affineTransform<double>(accelerationDefinition.value("mounting").toObject());
            if (accMounting.has_value()) {
                _accelerometerMounting = accMounting.value();
            }
        }

    }

    if (data.contains("accelerationFile")) {
        _accelerationFile = data.value("accelerationFile").toString();
    }

    if (data.contains("angularSpeedDefinition")) {

        QJsonObject angularSpeedDefinition = data.value("angularSpeedDefinition").toObject();

        if (angularSpeedDefinition.contains("time_delta")) {
            _angularSpeedDefinition.timeDelta = angularSpeedDefinition.value("time_delta").toDouble(0);
        } else {
            _angularSpeedDefinition.timeDelta = 0;
        }

        if (angularSpeedDefinition.contains("time_scale")) {
            _angularSpeedDefinition.timeScale = angularSpeedDefinition.value("time_scale").toDouble(1);
        } else {
            _angularSpeedDefinition.timeScale = 1;
        }

        if (angularSpeedDefinition.contains("angle_representation")) {

            QMetaEnum e = QMetaEnum::fromType<AngleRepresentation>();
            bool ok = true;
            int val = e.keysToValue(angularSpeedDefinition.value("angle_representation").toString().toLocal8Bit().data(), &ok);

            if (ok) {
                _angularSpeedDefinition.angle_representation = static_cast<AngleRepresentation>(val);
            } else {
                _angularSpeedDefinition.angle_representation = AxisAngle; //default
            }
        }
        if (angularSpeedDefinition.contains("angleUnit")) {

            QMetaEnum e = QMetaEnum::fromType<AngleUnits>();
            bool ok = true;
            int val = e.keysToValue(angularSpeedDefinition.value("angleUnit").toString().toLocal8Bit().data(), &ok);

            if (ok) {
                _angularSpeedDefinition.angleUnit = static_cast<AngleUnits>(val);
            } else {
                _angularSpeedDefinition.angleUnit = Degree; //default
            }
        }

        if (angularSpeedDefinition.contains("pos_t_col")) {
            _angularSpeedDefinition.w_t_col = angularSpeedDefinition.value("pos_t_col").toInt();
        }
        if (angularSpeedDefinition.contains("pos_x_col")) {
            _angularSpeedDefinition.w_x_col = angularSpeedDefinition.value("pos_x_col").toInt();
        }
        if (angularSpeedDefinition.contains("pos_y_col")) {
            _angularSpeedDefinition.w_y_col = angularSpeedDefinition.value("pos_y_col").toInt();
        }
        if (angularSpeedDefinition.contains("pos_z_col")) {
            _angularSpeedDefinition.w_z_col = angularSpeedDefinition.value("pos_z_col").toInt();
        }
        if (angularSpeedDefinition.contains("pos_w_col")) {
            _angularSpeedDefinition.w_w_col = angularSpeedDefinition.value("pos_w_col").toInt();
        }

        if (angularSpeedDefinition.contains("sign_x")) {
            _angularSpeedDefinition.sign_x = angularSpeedDefinition.value("sign_x").toInt();
        }
        if (angularSpeedDefinition.contains("sign_y")) {
            _angularSpeedDefinition.sign_y = angularSpeedDefinition.value("sign_y").toInt();
        }
        if (angularSpeedDefinition.contains("sign_z")) {
            _angularSpeedDefinition.sign_z = angularSpeedDefinition.value("sign_z").toInt();
        }
        if (angularSpeedDefinition.contains("sign_w")) {
            _angularSpeedDefinition.sign_w = angularSpeedDefinition.value("sign_w").toInt();
        }

        if (angularSpeedDefinition.contains("mounting")) {
            auto gyroMounting = json2affineTransform<double>(angularSpeedDefinition.value("mounting").toObject());
            if (gyroMounting.has_value()) {
                _gyroMounting = gyroMounting.value();
            }
        }

    }

    if (data.contains("angularSpeedFile")) {
        _angularSpeedFile = data.value("angularSpeedFile").toString();
    }

    if (data.contains("positionDefinition")) {

        QJsonObject positionDefinition = data.value("positionDefinition").toObject();

        if (positionDefinition.contains("crs_epsg")) {
            _positionDefinition.crs_epsg = positionDefinition.value("crs_epsg").toString("");
        }

        if (positionDefinition.contains("time_delta")) {
            _positionDefinition.timeDelta = positionDefinition.value("time_delta").toDouble(0);
        } else {
            _positionDefinition.timeDelta = 0;
        }

        if (positionDefinition.contains("time_scale")) {
            _positionDefinition.timeScale = positionDefinition.value("time_scale").toDouble(1);
        } else {
            _positionDefinition.timeScale = 1;
        }

        if (positionDefinition.contains("pos_t_col")) {
            _positionDefinition.pos_t_col = positionDefinition.value("pos_t_col").toInt(-1);
        }

        if (positionDefinition.contains("pos_x_col")) {
            _positionDefinition.pos_x_col = positionDefinition.value("pos_x_col").toInt(-1);
        }

        if (positionDefinition.contains("pos_y_col")) {
            _positionDefinition.pos_y_col = positionDefinition.value("pos_y_col").toInt(-1);
        }

        if (positionDefinition.contains("pos_z_col")) {
            _positionDefinition.pos_z_col = positionDefinition.value("pos_z_col").toInt(-1);
        }

    }

    if (data.contains("positionFile")) {
        _positionFile = data.value("positionFile").toString();
    }

    if (data.contains("orientationDefinition")) {

        QJsonObject orientationDefinition = data.value("orientationDefinition").toObject();

        if (orientationDefinition.contains("time_delta")) {
            _orientationDefinition.timeDelta = orientationDefinition.value("time_delta").toDouble(0);
        } else {
            _orientationDefinition.timeDelta = 0;
        }

        if (orientationDefinition.contains("time_scale")) {
            _orientationDefinition.timeScale = orientationDefinition.value("time_scale").toDouble(1);
        } else {
            _orientationDefinition.timeScale = 1;
        }

        if (orientationDefinition.contains("topocentric_convention")) {
            QMetaEnum e = QMetaEnum::fromType<TopocentricConvention>();
            bool ok = true;
            int val = e.keysToValue(orientationDefinition.value("topocentric_convention").toString().toLocal8Bit().data(), &ok);

            if (ok) {
                _orientationDefinition.topocentric_convention = static_cast<TopocentricConvention>(val);
            } else {
                _orientationDefinition.topocentric_convention = NED; //default
            }
        }
        if (orientationDefinition.contains("angle_representation")) {

            QMetaEnum e = QMetaEnum::fromType<AngleRepresentation>();
            bool ok = true;
            int val = e.keysToValue(orientationDefinition.value("angle_representation").toString().toLocal8Bit().data(), &ok);

            if (ok) {
                _orientationDefinition.angle_representation = static_cast<AngleRepresentation>(val);
            } else {
                _orientationDefinition.angle_representation = AxisAngle; //default
            }
        }
        if (orientationDefinition.contains("angleUnit")) {

            QMetaEnum e = QMetaEnum::fromType<AngleUnits>();
            bool ok = true;
            int val = e.keysToValue(orientationDefinition.value("angleUnit").toString().toLocal8Bit().data(), &ok);

            if (ok) {
                _orientationDefinition.angleUnit = static_cast<AngleUnits>(val);
            } else {
                _orientationDefinition.angleUnit = Degree; //default
            }
        }

        if (orientationDefinition.contains("pos_t_col")) {
            _orientationDefinition.orient_t_col = orientationDefinition.value("pos_t_col").toInt();
        }
        if (orientationDefinition.contains("pos_x_col")) {
            _orientationDefinition.orient_x_col = orientationDefinition.value("pos_x_col").toInt();
        }
        if (orientationDefinition.contains("pos_y_col")) {
            _orientationDefinition.orient_y_col = orientationDefinition.value("pos_y_col").toInt();
        }
        if (orientationDefinition.contains("pos_z_col")) {
            _orientationDefinition.orient_z_col = orientationDefinition.value("pos_z_col").toInt();
        }
        if (orientationDefinition.contains("pos_w_col")) {
            _orientationDefinition.orient_w_col = orientationDefinition.value("pos_w_col").toInt();
        }

        if (orientationDefinition.contains("sign_x")) {
            _orientationDefinition.sign_x = orientationDefinition.value("sign_x").toInt();
        }
        if (orientationDefinition.contains("sign_y")) {
            _orientationDefinition.sign_y = orientationDefinition.value("sign_y").toInt();
        }
        if (orientationDefinition.contains("sign_z")) {
            _orientationDefinition.sign_z = orientationDefinition.value("sign_z").toInt();
        }
        if (orientationDefinition.contains("sign_w")) {
            _orientationDefinition.sign_w = orientationDefinition.value("sign_w").toInt();
        }

    }

    if (data.contains("orientationFile")) {
        _orientationFile = data.value("orientationFile").toString();
    }

    if (data.contains("accelerometerId")) {
        _accelerometerId = data.value("accelerometerId").toInt();
    } else {
        _accelerometerId = 1;
    }

    if (data.contains("gyroId")) {
        _gyroId = data.value("gyroId").toInt();
    } else {
        _gyroId = 1;
    }

    if (data.contains("inputDataSeparators")) {
        QJsonArray seps = data.value("inputDataSeparators").toArray();

        _separators.clear();

        for (int i = 0; i < seps.size(); i++) {
            _separators.push_back(seps[i].toString());
        }
    }

    if (_separators.isEmpty()) { //default value
        _separators.push_back(",");
    }

    if (data.contains("commentPattern")) {
        _commentPattern = data.value("commentPattern").toString("#");

        if (_commentPattern.isEmpty()) { //default value
            _commentPattern = "#";
        }
    } else {
        _commentPattern = "#";
    }

    if (data.contains("optimizedTrajectory")) {
        QJsonValue val = data.value("optimizedTrajectory");

        if (val.isObject()) {

            QJsonObject jsonObj = val.toObject();

            DataBlock* optimizedTrajectory = getOptimizedDataTable();
            optimizedTrajectory->configureFromJson(jsonObj);
        }
    }

    if (data.contains("plateformDefinition")) {

        QJsonObject plateformDefinition = data.value("plateformDefinition").toObject();

        if (plateformDefinition.contains("boresight")) {
            QJsonArray axisRep = plateformDefinition.value("boresight").toArray();

            if (axisRep.size() == 3) {
                for (int i = 0; i < 3; i++) {
                    _plateformDefinition.boresight[i] = axisRep.at(i).toDouble();
                }
            }
        }

        if (plateformDefinition.contains("leverArm")) {
            QJsonArray leverArmRep = plateformDefinition.value("leverArm").toArray();

            if (leverArmRep.size() == 3) {
                for (int i = 0; i < 3; i++) {
                    _plateformDefinition.leverArm[i] = leverArmRep.at(i).toDouble();
                }
            }
        }
    }

}



void Trajectory::extendDataModel() {
    ItemDataModel::Category* sensorsIdsCat = _dataModel->addCategory(tr("Sensors Ids"));

    sensorsIdsCat->addCatProperty<int,
            Trajectory,
            false,
            ItemDataModel::ItemPropertyDescription::NoValueSignal>(
                tr("Accelerometer Id"),
                &Trajectory::accelerometerId,
                &Trajectory::setAccelerometerId,
                &Trajectory::accelerometerIdChanged);

    sensorsIdsCat->addCatProperty<int,
            Trajectory,
            false,
            ItemDataModel::ItemPropertyDescription::NoValueSignal>(
                tr("Gyroscope Id"),
                &Trajectory::gyroId,
                &Trajectory::setGyroId,
                &Trajectory::gyroIdChanged);


    ItemDataModel::Category* accelerometerOptIdsCat = _dataModel->addCategory(tr("Accelerometer Optimized Parameters"));

    accelerometerOptIdsCat->addCatProperty<floatParameter,
            Trajectory,
            true,
            ItemDataModel::ItemPropertyDescription::NoValueSignal>(
                tr("bias X"),
                &Trajectory::optAccelerometerBiasX,
                &Trajectory::setOptAccelerometerBiasX,
                &Trajectory::accelerometerBiasXChanged);

    accelerometerOptIdsCat->addCatProperty<floatParameter,
            Trajectory,
            true,
            ItemDataModel::ItemPropertyDescription::NoValueSignal>(
                tr("bias Y"),
                &Trajectory::optAccelerometerBiasY,
                &Trajectory::setOptAccelerometerBiasY,
                &Trajectory::accelerometerBiasYChanged);

    accelerometerOptIdsCat->addCatProperty<floatParameter,
            Trajectory,
            true,
            ItemDataModel::ItemPropertyDescription::NoValueSignal>(
                tr("bias Z"),
                &Trajectory::optAccelerometerBiasZ,
                &Trajectory::setOptAccelerometerBiasZ,
                &Trajectory::accelerometerBiasZChanged);
}

int Trajectory::gyroId() const
{
    return _gyroId;
}

void Trajectory::setGyroId(int newGyroId)
{
    if (_gyroId == newGyroId)
        return;
    _gyroId = newGyroId;
    Q_EMIT gyroIdChanged();
}

int Trajectory::accelerometerId() const
{
    return _accelerometerId;
}

void Trajectory::setAccelerometerId(int newAccelerometerId)
{
    if (_accelerometerId == newAccelerometerId)
        return;
    _accelerometerId = newAccelerometerId;
    Q_EMIT accelerometerIdChanged();
}

TrajectoryFactory::TrajectoryFactory(QObject* parent) :
    DataBlockFactory(parent)
{

}

QString TrajectoryFactory::TypeDescrName() const {
    return tr("Trajectory");
}
DataBlockFactory::FactorizableFlags TrajectoryFactory::factorizable() const {
    return DataBlockFactory::RootDataBlock;
}
DataBlock* TrajectoryFactory::factorizeDataBlock(Project *parent) const {
    return new Trajectory(parent);
}

QString TrajectoryFactory::itemClassName() const {
    return Trajectory::staticMetaObject.className();
}

} // namespace StereoVisionApp
