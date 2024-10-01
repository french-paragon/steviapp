#ifndef JSONCONVERSIONSUTILS_H
#define JSONCONVERSIONSUTILS_H

#include <QJsonObject>
#include <QJsonArray>

#include <StereoVision/geometry/rotations.h>

#include <optional>

namespace StereoVisionApp {

template<typename T>
inline QJsonObject affineTransform2json(StereoVision::Geometry::AffineTransform<T> const& transform) {

    QJsonObject ret;

    QJsonArray R;
    QJsonArray t;

    for (int i = 0; i < 3; i++) {
        QJsonArray R_row;

        for (int j = 0; j < 3; j++) {
            R_row.push_back(transform.R(i,j));
        }

        R.push_back(R_row);

        t.push_back(transform.t[i]);
    }

    ret.insert("R", R);
    ret.insert("t", t);

    return ret;
}

template<typename T>
std::optional<StereoVision::Geometry::AffineTransform<T>> json2affineTransform(QJsonObject const& obj) {

    StereoVision::Geometry::AffineTransform<T> ret;

    if (obj.contains("R") and obj.contains("t")) {

        QJsonArray R = obj.value("R").toArray();
        QJsonArray t = obj.value("t").toArray();

        for (int i = 0; i < 3; i++) {
            QJsonArray row = R.at(i).toArray();
            ret.R.row(i) << row.at(0).toDouble(), row.at(1).toDouble(), row.at(2).toDouble();
        }

        ret.t << t.at(0).toDouble(), t.at(1).toDouble(), t.at(2).toDouble();

    } else {
        return std::nullopt;
    }

    return ret;
}

}

#endif // JSONCONVERSIONSUTILS_H
