#include "trajectory.h"

#include <QFile>
#include <QRegularExpression>
#include <QJsonObject>
#include <QJsonArray>

#include <QMetaEnum>

#include "geo/localframes.h"

namespace StereoVisionApp {

Trajectory::Trajectory(Project* parent) :
    DataBlock(parent)
{
    _plateformDefinition.boresight.setConstant(0);
    _plateformDefinition.leverArm.setConstant(0);
}

std::vector<Trajectory::TimeCartesianBlock> Trajectory::loadPositionData() const {

    std::vector<TimeCartesianBlock> ret = loadPositionRawData();

    if (ret.empty()) {
        return std::vector<TimeCartesianBlock>();
    }

    if (_positionDefinition.crs_epsg.isEmpty()) {
        return std::vector<TimeCartesianBlock>();
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

    std::vector<TimeTrajectoryBlock> data = loadTrajectoryData();

    if (data.empty()) {
        return std::nullopt;
    }

    return TimeTrajectorySequence(std::move(data));
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
        PoseType poseTransform(StereoVision::Geometry::inverseRodriguezFormula(transform.R), transform.t);

        for (int i = 0; i < data.size(); i++) {

            PoseType pose = data[i].val; //sensor 2 ecef
            pose =  poseTransform * pose; // plateform 2 project local

            data[i].val = pose;
        }

    }

    return TimeTrajectorySequence(std::move(data));

}

bool Trajectory::geoReferenceSupportActive() const  {
    return !_positionDefinition.crs_epsg.isEmpty();
}

Eigen::Array<float,3, Eigen::Dynamic> Trajectory::getLocalPointsEcef() const  {

    std::vector<TimeCartesianBlock> positionData = loadPositionData();

    Eigen::Array<float,3, Eigen::Dynamic> ret;

    ret.resize(3,1);
    ret.col(0) = positionData[0].val.x();
    ret.col(1) = positionData[0].val.y();
    ret.col(2) = positionData[0].val.z();

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

QJsonObject Trajectory::encodeJson() const {

    QJsonObject obj;

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

    QMetaEnum e = QMetaEnum::fromType<TopocentricConvention>();
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

    QJsonArray separatorsStrings;

    for (int i = 0; i < _separators.size(); i++) {
        separatorsStrings.push_back(QJsonValue(_separators[i]));
    }

    obj.insert("inputDataSeparators", separatorsStrings);

    obj.insert("commentPattern", _commentPattern);

    return obj;

}
void Trajectory::configureFromJson(QJsonObject const& data) {

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

    if (data.contains("commentPattern")) {
        _commentPattern = data.value("commentPattern").toString("#");
    } else {
        _commentPattern = "#";
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
