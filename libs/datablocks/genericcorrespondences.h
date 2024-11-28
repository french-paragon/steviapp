#ifndef STEREOVISIONAPP_GENERICCORRESPONDENCES_H
#define STEREOVISIONAPP_GENERICCORRESPONDENCES_H

#include <QString>
#include <QStringList>
#include <QRegExp>
#include <QPair>

#include <optional>
#include <variant>

namespace StereoVisionApp {

namespace Correspondences {

enum Types {
    UV = 0, //UV coordinates on a 2D grid
    XY = 1, //XY coordinates in a 2D space, we do not re-use UV in case some objects whant to distinguish between a grid (e.g. pixel space) and local space.
    GEOXY = 2, //X,Y coordinates of a projected points in a geo system
    XYZ = 3, //X,Y,Z 3D point in a 3D space
    GEOXYZ = 4, //X,Y,Z coordinates of a point in a geo system
    T = 5, //Time, coordinate of a point in time
    Line2D = 6, //lie anywhere on a 2D line
    Line3D = 7, //lie anywhere on a 3D line
    Plane3D = 8, //lie anywhere on a 3D plane
    XYT = 9, //lie at a XY coordinate in a 2D space at a specified time
    XYZT = 10, //lie at a XYZ coordinate in a 3D space at a specified time
    PRIOR = 11, //to nothing, can be usefull to use a correspondance as a singleton for a prior
    INVALID = 12 //to nothing, just an invalid correspondance
};

inline Types correspTypeFromString(QString const& str) {

    QString lower = str.toLower().trimmed();

    if (lower == "uv") {
        return Types::UV;
    }

    if (lower == "xy") {
        return Types::XY;
    }

    if (lower == "geoxy") {
        return Types::GEOXY;
    }

    if (lower == "t") {
        return Types::T;
    }

    if (lower == "line2d") {
        return Types::Line2D;
    }

    if (lower == "line3d") {
        return Types::Line3D;
    }

    if (lower == "plane3d") {
        return Types::Plane3D;
    }

    if (lower == "xyt") {
        return Types::XYT;
    }

    if (lower == "xyzt") {
        return Types::XYZT;
    }

    if (lower == "prior") {
        return Types::PRIOR;
    }

    if (lower == "invalid") {
        return Types::INVALID;
    }

    return Types::INVALID;
}

inline QString correspTypeToString(Types correspT) {
    switch (correspT) {
    case UV:
        return "UV";
    case XY:
        return "XY";
    case GEOXY:
        return "GEOXY";
    case XYZ:
        return "XYZ";
    case GEOXYZ:
        return "GEOXYZ";
    case T:
        return "T";
    case Line2D:
        return "Line2D";
    case Line3D:
        return "Line3D";
    case Plane3D:
        return "Plane3D";
    case XYT:
        return "XYT";
    case XYZT:
        return "XYZT";
    case PRIOR:
        return "PRIOR";
    default:
        break;
    }
    return "INVALID";
}

/*!
 * \brief The CorrespBlock class represent a part of a correspondance.
 *
 * Default implementation represent an empty correspondance, used for example for the PRIOR and INVALID cases
 */
template<Types C>
struct Typed {

    static constexpr Types CorrespType = C;

    static std::optional<Typed> fromString(QString const& str) {
        return Typed();
    }

    QString toStr() const {
        return correspTypeToString(C);
    };
};

namespace Internal {
template<bool withId, bool withCRS, int nFloats, int nOptFloats>
    struct floatListReader {

        qint64 blockId;
        QString crsInfos;
        std::array<float, nFloats> floatsVals;
        std::array<std::optional<float>, nOptFloats> optFloatsVals;

        static std::optional<floatListReader> fromString(QString const& str) {

            constexpr int size_delta = (withId) ? ((withCRS) ? 2 : 1) : ((withCRS) ? 1 : 0);

            QStringList split = str.split(QRegExp("\\s+"), Qt::SkipEmptyParts);

            if (split.size() != nFloats+size_delta and split.size() != nFloats+nOptFloats+size_delta) { return std::nullopt; }

            bool ok;

            qint64 blockId;

            if (withId) {
                blockId = split[0].toInt(&ok);
                if (!ok) { return std::nullopt; }
            }

            QString crsInfos;

            if (withCRS) {
                crsInfos = split[(withId) ? 1 : 0];
            }

            std::array<float, nFloats> floatsVals;

            for (int i = 0; i < nFloats; i++) {
                floatsVals[i] = split[i+size_delta].toFloat(&ok);
                if (!ok) { return std::nullopt; }
            }

            std::array<std::optional<float>, nOptFloats> optFloatsVals;

            for (int i = 0; i < nOptFloats; i++) {
                optFloatsVals[i] = std::nullopt;
            }

            if (split.size() != nFloats+nOptFloats+size_delta) {

                for (int i = 0; i < nOptFloats; i++) {
                    optFloatsVals[i] = split[i+nFloats+size_delta].toFloat(&ok);
                    if (!ok) { return std::nullopt; }
                }
            }

            floatListReader ret;
            ret.blockId = blockId;
            ret.crsInfos = crsInfos;
            ret.floatsVals = floatsVals;
            ret.optFloatsVals = optFloatsVals;

            return ret;
        }
    };
}

template<>
struct Typed<Types::UV> {

    static constexpr Types CorrespType = Types::UV;

    qint64 blockId;
    float u;
    float v;
    std::optional<float> sigmaU;
    std::optional<float> sigmaV;

    static std::optional<Typed> fromString(QString const& str) {

        using floatListReader = Internal::floatListReader<true, false, 2, 2>;

        std::optional<floatListReader> data = floatListReader::fromString(str);

        if (!data.has_value()) {
            return std::nullopt;
        }

        Typed ret;

        ret.blockId = data.value().blockId;
        ret.u = data.value().floatsVals[0];
        ret.v = data.value().floatsVals[1];
        ret.sigmaU = data.value().optFloatsVals[0];
        ret.sigmaV = data.value().optFloatsVals[1];

        return ret;
    }

    QString toStr() const {
        QString out = QString(" %1 %2 %3").arg(blockId).arg(u, 0,'f',2).arg(v, 0,'f',2);
        if (sigmaU.has_value() and sigmaV.has_value()) {
            out += QString(" %1 %2").arg(sigmaU.value(), 0,'f',2).arg(sigmaV.value(), 0,'f',2);
        }
        return correspTypeToString(CorrespType) + out;
    };
};


template<>
struct Typed<Types::XY> {

    static constexpr Types CorrespType = Types::XY;

    qint64 blockId;
    float x;
    float y;
    std::optional<float> sigmaX;
    std::optional<float> sigmaY;

    static std::optional<Typed> fromString(QString const& str) {

        using floatListReader = Internal::floatListReader<true, false, 2, 2>;

        std::optional<floatListReader> data = floatListReader::fromString(str);

        if (!data.has_value()) {
            return std::nullopt;
        }

        Typed ret;

        ret.blockId = data.value().blockId;
        ret.x = data.value().floatsVals[0];
        ret.y = data.value().floatsVals[1];
        ret.sigmaX = data.value().optFloatsVals[0];
        ret.sigmaY = data.value().optFloatsVals[1];

        return ret;
    }

    QString toStr() const {
        QString out = QString(" %1 %2 %3").arg(blockId).arg(x, 0,'f',-1).arg(y, 0,'f',-1);
        if (sigmaX.has_value() and sigmaY.has_value()) {
            out += QString(" %1 %2").arg(sigmaX.value(), 0,'f',-1).arg(sigmaY.value(), 0,'f',-1);
        }
        return correspTypeToString(CorrespType) + out;
    };
};

template<>
struct Typed<Types::GEOXY> {

    static constexpr Types CorrespType = Types::GEOXY;

    QString crsInfos;
    float x;
    float y;
    std::optional<float> sigmaX;
    std::optional<float> sigmaY;

    static std::optional<Typed> fromString(QString const& str) {

        using floatListReader = Internal::floatListReader<false, true, 2, 2>;

        std::optional<floatListReader> data = floatListReader::fromString(str);

        if (!data.has_value()) {
            return std::nullopt;
        }

        Typed ret;

        ret.crsInfos = data.value().crsInfos;
        ret.x = data.value().floatsVals[0];
        ret.y = data.value().floatsVals[1];
        ret.sigmaX = data.value().optFloatsVals[0];
        ret.sigmaY = data.value().optFloatsVals[1];

        return ret;
    }

    QString toStr() const {
        QString out = QString(" %1 %2 %3").arg(crsInfos).arg(x, 0,'f',-1).arg(y, 0,'f',-1);
        if (sigmaX.has_value() and sigmaY.has_value()) {
            out += QString(" %1 %2").arg(sigmaX.value(), 0,'f',-1).arg(sigmaY.value(), 0,'f',-1);
        }
        return correspTypeToString(CorrespType) + out;
    };
};

template<>
struct Typed<Types::XYZ> {

    static constexpr Types CorrespType = Types::XYZ;

    qint64 blockId;
    float x;
    float y;
    float z;
    std::optional<float> sigmaX;
    std::optional<float> sigmaY;
    std::optional<float> sigmaZ;

    static std::optional<Typed> fromString(QString const& str) {

        using floatListReader = Internal::floatListReader<true, false, 3, 3>;

        std::optional<floatListReader> data = floatListReader::fromString(str);

        if (!data.has_value()) {
            return std::nullopt;
        }

        Typed ret;

        ret.blockId = data.value().blockId;
        ret.x = data.value().floatsVals[0];
        ret.y = data.value().floatsVals[1];
        ret.z = data.value().floatsVals[2];
        ret.sigmaX = data.value().optFloatsVals[0];
        ret.sigmaY = data.value().optFloatsVals[1];
        ret.sigmaZ = data.value().optFloatsVals[2];

        return ret;
    }

    QString toStr() const {
        QString out = QString(" %1 %2 %3 %4").arg(blockId).arg(x, 0,'f',-1).arg(y, 0,'f',-1).arg(z, 0,'f',-1);
        if (sigmaX.has_value() and sigmaY.has_value() and sigmaZ.has_value()) {
            out += QString(" %1 %2 %3").arg(sigmaX.value(), 0,'f',-1).arg(sigmaY.value(), 0,'f',-1).arg(sigmaZ.value(), 0,'f',-1);
        }
        return correspTypeToString(CorrespType) + out;
    };
};

template<>
struct Typed<Types::GEOXYZ> {

    static constexpr Types CorrespType = Types::GEOXYZ;

    QString crsInfos;
    float x;
    float y;
    float z;
    std::optional<float> sigmaX;
    std::optional<float> sigmaY;
    std::optional<float> sigmaZ;

    static std::optional<Typed> fromString(QString const& str) {

        using floatListReader = Internal::floatListReader<false, true, 3, 3>;

        std::optional<floatListReader> data = floatListReader::fromString(str);

        if (!data.has_value()) {
            return std::nullopt;
        }

        Typed ret;

        ret.crsInfos = data.value().crsInfos;
        ret.x = data.value().floatsVals[0];
        ret.y = data.value().floatsVals[1];
        ret.z = data.value().floatsVals[2];
        ret.sigmaX = data.value().optFloatsVals[0];
        ret.sigmaY = data.value().optFloatsVals[1];
        ret.sigmaZ = data.value().optFloatsVals[2];

        return ret;
    }

    QString toStr() const {
        QString out = QString(" %1 %2 %3 %4").arg(crsInfos).arg(x, 0,'f',-1).arg(y, 0,'f',-1).arg(z, 0,'f',-1);
        if (sigmaX.has_value() and sigmaY.has_value() and sigmaZ.has_value()) {
            out += QString(" %1 %2 %3").arg(sigmaX.value(), 0,'f',-1).arg(sigmaY.value(), 0,'f',-1).arg(sigmaZ.value(), 0,'f',-1);
        }
        return correspTypeToString(CorrespType) + out;
    };
};

template<>
struct Typed<Types::T> {

    static constexpr Types CorrespType = Types::T;

    qint64 blockId;
    float t;
    std::optional<float> sigmaT;

    static std::optional<Typed> fromString(QString const& str) {

        using floatListReader = Internal::floatListReader<true, false, 1, 1>;

        std::optional<floatListReader> data = floatListReader::fromString(str);

        if (!data.has_value()) {
            return std::nullopt;
        }

        Typed ret;

        ret.blockId = data.value().blockId;
        ret.t = data.value().floatsVals[0];
        ret.sigmaT = data.value().optFloatsVals[0];

        return ret;
    }

    QString toStr() const {
        QString out = QString(" %1 %2").arg(blockId).arg(t, 0,'f',-1);
        if (sigmaT.has_value()) {
            out += QString(" %1").arg(sigmaT.value(), 0,'f',-1);
        }
        return correspTypeToString(CorrespType) + out;
    };
};

template<>
struct Typed<Types::XYT> {

    static constexpr Types CorrespType = Types::XYT;

    qint64 blockId;
    float x;
    float y;
    float t;
    std::optional<float> sigmaX;
    std::optional<float> sigmaY;
    std::optional<float> sigmaT;

    static std::optional<Typed> fromString(QString const& str) {

        using floatListReader = Internal::floatListReader<true, false, 3, 3>;

        std::optional<floatListReader> data = floatListReader::fromString(str);

        if (!data.has_value()) {
            return std::nullopt;
        }

        Typed ret;

        ret.blockId = data.value().blockId;
        ret.x = data.value().floatsVals[0];
        ret.y = data.value().floatsVals[1];
        ret.t = data.value().floatsVals[2];
        ret.sigmaX = data.value().optFloatsVals[0];
        ret.sigmaY = data.value().optFloatsVals[1];
        ret.sigmaT = data.value().optFloatsVals[2];

        return ret;
    }

    QString toStr() const {
        QString out = QString(" %1 %2 %3 %4").arg(blockId).arg(x, 0,'f',-1).arg(y, 0,'f',-1).arg(t, 0,'f',-1);
        if (sigmaX.has_value() and sigmaY.has_value() and sigmaT.has_value()) {
            out += QString(" %1 %2 %3").arg(sigmaX.value(), 0,'f',-1).arg(sigmaY.value(), 0,'f',-1).arg(sigmaT.value(), 0,'f',-1);
        }
        return correspTypeToString(CorrespType) + out;
    };
};

template<>
struct Typed<Types::XYZT> {

    static constexpr Types CorrespType = Types::XYZT;

    qint64 blockId;
    float x;
    float y;
    float z;
    float t;
    std::optional<float> sigmaX;
    std::optional<float> sigmaY;
    std::optional<float> sigmaZ;
    std::optional<float> sigmaT;

    static std::optional<Typed> fromString(QString const& str) {

        using floatListReader = Internal::floatListReader<true, false, 4, 4>;

        std::optional<floatListReader> data = floatListReader::fromString(str);

        if (!data.has_value()) {
            return std::nullopt;
        }

        Typed ret;

        ret.blockId = data.value().blockId;
        ret.x = data.value().floatsVals[0];
        ret.y = data.value().floatsVals[1];
        ret.z = data.value().floatsVals[2];
        ret.t = data.value().floatsVals[3];
        ret.sigmaX = data.value().optFloatsVals[0];
        ret.sigmaY = data.value().optFloatsVals[1];
        ret.sigmaZ = data.value().optFloatsVals[2];
        ret.sigmaT = data.value().optFloatsVals[3];

        return ret;
    }

    QString toStr() const {
        QString out = QString(" %1 %2 %3 %4 %5").arg(blockId).arg(x, 0,'f',-1).arg(y, 0,'f',-1).arg(z, 0,'f',-1).arg(t, 0,'f',-1);
        if (sigmaX.has_value() and sigmaY.has_value() and sigmaZ.has_value() and sigmaT.has_value()) {
            out += QString(" %1 %2 %3 %4").arg(sigmaX.value(), 0,'f',-1).arg(sigmaY.value(), 0,'f',-1).arg(sigmaZ.value(), 0,'f',-1).arg(sigmaT.value(), 0,'f',-1);
        }
        return correspTypeToString(CorrespType) + out;
    };
};

//This block represent a 2D line in a local 2D coordinate system. The line is represented by two points (x1,y1),(x2,y2)
template<>
struct Typed<Types::Line2D> {

    static constexpr Types CorrespType = Types::Line2D;

    qint64 blockId;
    float x1;
    float y1;
    float x2;
    float y2;
    std::optional<float> sigmaX1;
    std::optional<float> sigmaY1;
    std::optional<float> sigmaX2;
    std::optional<float> sigmaY2;

    static std::optional<Typed> fromString(QString const& str) {

        using floatListReader = Internal::floatListReader<true, false, 4, 4>;

        std::optional<floatListReader> data = floatListReader::fromString(str);

        if (!data.has_value()) {
            return std::nullopt;
        }

        Typed ret;

        ret.blockId = data.value().blockId;
        ret.x1 = data.value().floatsVals[0];
        ret.y1 = data.value().floatsVals[1];
        ret.x2 = data.value().floatsVals[2];
        ret.y2 = data.value().floatsVals[3];
        ret.sigmaX1 = data.value().optFloatsVals[0];
        ret.sigmaY1 = data.value().optFloatsVals[1];
        ret.sigmaX2 = data.value().optFloatsVals[2];
        ret.sigmaY2 = data.value().optFloatsVals[3];

        return ret;
    }

    QString toStr() const {
        QString out = QString(" %1 %2 %3 %4 %5").arg(blockId).arg(x1, 0,'f',-1).arg(y1, 0,'f',-1).arg(x2, 0,'f',-1).arg(y2, 0,'f',-1);
        if (sigmaX1.has_value() and sigmaY1.has_value() and sigmaX2.has_value() and sigmaY2.has_value()) {
            out += QString(" %1 %2 %3 %4").arg(sigmaX1.value(), 0,'f',-1).arg(sigmaY1.value(), 0,'f',-1).arg(sigmaX2.value(), 0,'f',-1).arg(sigmaY2.value(), 0,'f',-1);
        }
        return correspTypeToString(CorrespType) + out;
    };
};

//This block represent a 3D line in a local 3D coordinate system. The line is represented by two points (x1,y1,z1),(x2,y2,z2)
template<>
struct Typed<Types::Line3D> {

    static constexpr Types CorrespType = Types::Line3D;

    qint64 blockId;
    float x1;
    float y1;
    float z1;
    float x2;
    float y2;
    float z2;
    std::optional<float> sigmaX1;
    std::optional<float> sigmaY1;
    std::optional<float> sigmaZ1;
    std::optional<float> sigmaX2;
    std::optional<float> sigmaY2;
    std::optional<float> sigmaZ2;

    static std::optional<Typed> fromString(QString const& str) {

        using floatListReader = Internal::floatListReader<true, false, 6, 6>;

        std::optional<floatListReader> data = floatListReader::fromString(str);

        if (!data.has_value()) {
            return std::nullopt;
        }

        Typed ret;

        ret.blockId = data.value().blockId;
        ret.x1 = data.value().floatsVals[0];
        ret.y1 = data.value().floatsVals[1];
        ret.z1 = data.value().floatsVals[2];
        ret.x2 = data.value().floatsVals[3];
        ret.y2 = data.value().floatsVals[4];
        ret.z2 = data.value().floatsVals[5];
        ret.sigmaX1 = data.value().optFloatsVals[0];
        ret.sigmaY1 = data.value().optFloatsVals[1];
        ret.sigmaZ1 = data.value().optFloatsVals[2];
        ret.sigmaX2 = data.value().optFloatsVals[3];
        ret.sigmaY2 = data.value().optFloatsVals[4];
        ret.sigmaZ2 = data.value().optFloatsVals[5];

        return ret;
    }

    QString toStr() const {
        QString out = QString(" %1 %2 %3 %4 %5 %6 %7").arg(blockId)
                .arg(x1, 0,'f',-1).arg(y1, 0,'f',-1).arg(z1, 0,'f',-1)
                .arg(x2, 0,'f',-1).arg(y2, 0,'f',-1).arg(z2, 0,'f',-1);

        if (sigmaX1.has_value() and sigmaY1.has_value() and sigmaZ1.has_value()
                and sigmaX2.has_value() and sigmaY2.has_value() and sigmaZ2.has_value()) {

            out += QString(" %1 %2 %3 %4 %5 %6").arg(sigmaX1.value(), 0,'f',-1).arg(sigmaY1.value(), 0,'f',-1).arg(sigmaZ1.value(), 0,'f',-1)
                    .arg(sigmaX2.value(), 0,'f',-1).arg(sigmaY2.value(), 0,'f',-1).arg(sigmaZ2.value(), 0,'f',-1);
        }
        return correspTypeToString(CorrespType) + out;
    };
};

//This block represent a 3D plane in a local 3D coordinate system. The plane is represented by the normal vector (x,y,z) and a constant c.
template<>
struct Typed<Types::Plane3D> {

    static constexpr Types CorrespType = Types::Plane3D;

    qint64 blockId;
    float x;
    float y;
    float z;
    float c;
    std::optional<float> sigmaX;
    std::optional<float> sigmaY;
    std::optional<float> sigmaZ;
    std::optional<float> sigmaC;

    static std::optional<Typed> fromString(QString const& str) {

        using floatListReader = Internal::floatListReader<true, false, 4, 4>;

        std::optional<floatListReader> data = floatListReader::fromString(str);

        if (!data.has_value()) {
            return std::nullopt;
        }

        Typed ret;

        ret.blockId = data.value().blockId;
        ret.x = data.value().floatsVals[0];
        ret.y = data.value().floatsVals[1];
        ret.z = data.value().floatsVals[2];
        ret.c = data.value().floatsVals[3];
        ret.sigmaX = data.value().optFloatsVals[0];
        ret.sigmaY = data.value().optFloatsVals[1];
        ret.sigmaZ = data.value().optFloatsVals[2];
        ret.sigmaC = data.value().optFloatsVals[3];

        return ret;
    }

    QString toStr() const {
        QString out = QString(" %1 %2 %3 %4 %5").arg(blockId).arg(x, 0,'f',-1).arg(y, 0,'f',-1).arg(z, 0,'f',-1).arg(c, 0,'f',-1);
        if (sigmaX.has_value() and sigmaY.has_value() and sigmaZ.has_value() and sigmaC.has_value()) {
            out += QString(" %1 %2 %3 %4").arg(sigmaX.value(), 0,'f',-1).arg(sigmaY.value(), 0,'f',-1).arg(sigmaZ.value(), 0,'f',-1).arg(sigmaC.value(), 0,'f',-1);
        }
        return correspTypeToString(CorrespType) + out;
    };
};

using Generic = std::variant<Typed<Types::UV>,
Typed<Types::XY>,
Typed<Types::GEOXY>,
Typed<Types::XYZ>,
Typed<Types::GEOXYZ>,
Typed<Types::T>,
Typed<Types::XYT>,
Typed<Types::XYZT>,
Typed<Types::Line2D>,
Typed<Types::Line3D>,
Typed<Types::Plane3D>,
Typed<Types::PRIOR>,
Typed<Types::INVALID>>;

Generic GenericFromString(QString const& str);
QString GenericToString(Generic const& corresp);

/*!
 * \brief getCorrespDatablockId return the id of the datablock the correspondance is attached to
 * \param corresp the CorrespBlock
 * \return the id of the datablock attached to the correspondance, or -1 if the corresp is PRIOR, INVALID or any type of correspondance without datablock (e.g. geo references).
 */
template<Types C>
inline qint64 getDatablockId(Typed<C> const& corresp) {
    return -1;
}

template<>
inline qint64 getDatablockId<Types::UV>(Typed<Types::UV> const& corresp) {
    return corresp.blockId;
}

template<>
inline qint64 getDatablockId<Types::XY>(Typed<Types::XY> const& corresp) {
    return corresp.blockId;
}

template<>
inline qint64 getDatablockId<Types::XYZ>(Typed<Types::XYZ> const& corresp) {
    return corresp.blockId;
}

template<>
inline qint64 getDatablockId<Types::XYT>(Typed<Types::XYT> const& corresp) {
    return corresp.blockId;
}

template<>
inline qint64 getDatablockId<Types::XYZT>(Typed<Types::XYZT> const& corresp) {
    return corresp.blockId;
}

template<>
inline qint64 getDatablockId<Types::Line2D>(Typed<Types::Line2D> const& corresp) {
    return corresp.blockId;
}

template<>
inline qint64 getDatablockId<Types::Line3D>(Typed<Types::Line3D> const& corresp) {
    return corresp.blockId;
}

template<>
inline qint64 getDatablockId<Types::Plane3D>(Typed<Types::Plane3D> const& corresp) {
    return corresp.blockId;
}

inline qint64 getDatablockId(Generic const& corresp) {
    return std::visit([](auto const& arg){ return getDatablockId(arg); }, corresp);
}

template<Types C1, Types C2>
struct TypedPair {
    Typed<C1> c1;
    Typed<C2> c2;
};

struct GenericPair {
    Generic c1;
    Generic c2;

    inline bool isValid() const {
        return !std::holds_alternative<Typed<Types::INVALID>>(c1) and !std::holds_alternative<Typed<Types::INVALID>>(c2);
    }

    inline bool hasDatablockId(qint64 id) const {
        return getDatablockId(c1) == id or getDatablockId(c2) == id;
    }

    inline QPair<qint64,qint64> datablocksIds() const {
        return QPair(getDatablockId(c1), getDatablockId(c2));
    }

    template<Types C1, Types C2>
    bool holdsCorrespondancesType() const {
        return (std::holds_alternative<Typed<C1>>(c1) and std::holds_alternative<Typed<C2>>(c2)) or
                (std::holds_alternative<Typed<C2>>(c1) and std::holds_alternative<Typed<C1>>(c2));
    }

    template<Types C1, Types C2>
    std::optional<TypedPair<C1,C2>> getTypedPair() const {
        if (holdsCorrespondancesType<C1,C2>()) {
            if (std::holds_alternative<Typed<C1>>(c1)) {
                return TypedPair<C1,C2>{std::get<Typed<C1>>(c1),std::get<Typed<C2>>(c2)};
            } else {
                return TypedPair<C1,C2>{std::get<Typed<C1>>(c2),std::get<Typed<C2>>(c1)};
            }
        }
        return std::nullopt;
    };

    static inline std::optional<GenericPair> fromString (QString const& str) {
        QStringList split = str.split(",");

        if (split.size() != 2) {
            return std::nullopt;
        }

        GenericPair pair;

        pair.c1 = GenericFromString(split[0]);
        pair.c2 = GenericFromString(split[1]);

        return pair;
    }

    inline QString toString() const {
        return GenericToString(c1) + "," + GenericToString(c2);
    }

};

} // GenericCorrespondences

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_GENERICCORRESPONDENCES_H
