#include "trajectory.h"

#include <QFile>
#include <QRegularExpression>
#include <QJsonObject>
#include <QJsonArray>

#include <QMetaEnum>

#include "geo/localframes.h"

#include "datablocks/datatable.h"

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
}

std::vector<Trajectory::TimeCartesianBlock> Trajectory::loadAngularSpeedRawData() const {

    QFile angspdFile(_angularSpeedFile);

    if (!angspdFile.open(QIODevice::ReadOnly)) {
        return std::vector<Trajectory::TimeCartesianBlock>();
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

std::vector<Trajectory::TimeCartesianBlock> Trajectory::loadAccelerationRawData() const {

    QFile accFile(_accelerationFile);

    if (!accFile.open(QIODevice::ReadOnly)) {
        return std::vector<Trajectory::TimeCartesianBlock>();
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

std::vector<Trajectory::TimeCartesianBlock> Trajectory::loadPositionData() const {

    std::vector<TimeCartesianBlock> ret = loadPositionRawData();

    if (ret.empty()) {
        return std::vector<TimeCartesianBlock>();
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
        return std::vector<TimeCartesianBlock>();
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


std::optional<Trajectory::TimeCartesianSequence> Trajectory::loadAngularSpeedSequence() const {

    std::vector<TimeCartesianBlock> data = loadAngularSpeedRawData();

    if (data.empty()) {
        return std::nullopt;
    }

    return IndexedTimeSequence<Eigen::Vector3d, double>(std::move(data));

}

std::optional<Trajectory::TimeCartesianSequence> Trajectory::loadAccelerationSequence() const {

    std::vector<TimeCartesianBlock> data = loadAccelerationRawData();

    if (data.empty()) {
        return std::nullopt;
    }

    return IndexedTimeSequence<Eigen::Vector3d, double>(std::move(data));

}

std::optional<Trajectory::TimeCartesianSequence> Trajectory::loadOrientationSequence() const {
    std::vector<TimeCartesianBlock> data = loadOrientationRawData();

    if (data.empty()) {
        return std::nullopt;
    }

    return IndexedTimeSequence<Eigen::Vector3d, double>(std::move(data));

}

std::optional<Trajectory::TimeCartesianSequence> Trajectory::loadPositionSequence() const {
    std::vector<TimeCartesianBlock> data = loadPositionData();

    if (data.empty()) {
        return std::nullopt;
    }

    return IndexedTimeSequence<Eigen::Vector3d, double>(std::move(data));
}

std::vector<Eigen::Vector3f> Trajectory::loadTrajectoryPathInProjectLocalFrame() const {

    std::optional<StereoVision::Geometry::AffineTransform<float>> transform2projFrame = std::nullopt;

    Project* proj = getProject();

    if (proj != nullptr) {
        if (proj->hasLocalCoordinateFrame()) {
            transform2projFrame = proj->ecef2local();
        }
    }

    std::vector<TimeCartesianBlock> data = loadPositionData();

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
std::vector<StereoVision::Geometry::AffineTransform<float>> Trajectory::loadTrajectoryInProjectLocalFrame(bool optimized) const {

    if (optimized) {

        if (_optimizedTrajectoryData == nullptr) {
            return std::vector<StereoVision::Geometry::AffineTransform<float>>();
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

    std::vector<TimeTrajectoryBlock> data = loadTrajectoryData();

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

std::vector<Trajectory::TimeCartesianBlock> Trajectory::loadPositionRawData() const {

    QFile trajFile(_positionFile);

    if (!trajFile.open(QIODevice::ReadOnly)) {
        return std::vector<Trajectory::TimeCartesianBlock>();
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
std::vector<Trajectory::TimeCartesianBlock> Trajectory::loadOrientationRawData() const {

    QFile trajFile(_orientationFile);

    if (!trajFile.open(QIODevice::ReadOnly)) {
        return std::vector<Trajectory::TimeCartesianBlock>();
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

std::vector<Trajectory::TimeTrajectoryBlock> Trajectory::loadTrajectoryData() const {

    //TODO: generalize this function with interpolation
    std::vector<TimeCartesianBlock> posECEF = loadPositionData();
    std::vector<TimeCartesianBlock> orientationRaw = loadOrientationRawData();

    if (posECEF.size() != orientationRaw.size()) {
        return std::vector<Trajectory::TimeTrajectoryBlock>();
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
        return std::vector<Trajectory::TimeTrajectoryBlock>();
    }

    std::vector<StereoVision::Geometry::AffineTransform<double>>& localFrames = optLocalFrames.value();

    if (localFrames.size() != posECEF.size()) {
        return std::vector<Trajectory::TimeTrajectoryBlock>();
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

std::optional<Trajectory::TimeTrajectorySequence> Trajectory::loadTrajectorySequence() const {

    if (_trajectoryCache.has_value()) {
        return _trajectoryCache;
    }

    std::vector<TimeTrajectoryBlock> data = loadTrajectoryData();

    if (data.empty()) {
        return std::nullopt;
    }

    _trajectoryCache = TimeTrajectorySequence(std::move(data));

    return _trajectoryCache;
}

std::optional<Trajectory::TimeTrajectorySequence> Trajectory::loadTrajectoryProjectLocalFrameSequence() const {

    std::optional<StereoVision::Geometry::AffineTransform<float>> transform2projFrame = std::nullopt;

    Project* proj = getProject();

    if (proj != nullptr) {
        if (proj->hasLocalCoordinateFrame()) {
            transform2projFrame = proj->ecef2local();
        }
    }

    std::vector<TimeTrajectoryBlock> data = loadTrajectoryData();

    if (data.empty()) {
        return std::nullopt;
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

std::optional<Trajectory::TimeTrajectorySequence> Trajectory::optimizedTrajectory() const {

    if (_optimizedTrajectoryData == nullptr) {
        return std::nullopt;
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

    std::vector<TimeCartesianBlock> positionData = loadPositionData();

    Eigen::Array<float,3, Eigen::Dynamic> ret;

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
