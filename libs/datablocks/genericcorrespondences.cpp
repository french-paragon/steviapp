#include "genericcorrespondences.h"

#include <proj.h>

namespace StereoVisionApp {

namespace Correspondences {

namespace Internal {
template<Types C>
Generic dataToBlock(QString const& data) {
    auto block = Typed<C>::fromString(data);
    if (block.has_value()) {
        return block.value();
    }
    return Typed<Types::INVALID>();
};
}

Generic GenericFromString(QString const& str) {

    int idxNSpace = -1;
    int idxSpace = -1;

    for (int i = 0; i < str.size(); i++) {
        if (idxNSpace < 0 and !str[i].isSpace()) {
            idxNSpace = i;
        } else if (idxNSpace >= 0) {
            if (str[i].isSpace()) {
                idxSpace = i;
                break;
            }
        }
    }

    Types type = correspTypeFromString(str.mid(0,idxSpace));
    QString data = (idxSpace < 0) ? "" : str.mid(idxSpace);

    switch (type) {
    case UV:
        return Internal::dataToBlock<Types::UV>(data);
    case XY:
        return Internal::dataToBlock<Types::XY>(data);
    case GEOXY:
        return Internal::dataToBlock<Types::GEOXY>(data);
    case XYZ:
        return Internal::dataToBlock<Types::XYZ>(data);;
    case GEOXYZ:
        return Internal::dataToBlock<Types::GEOXYZ>(data);
    case T:
        return Internal::dataToBlock<Types::T>(data);
    case Line2D:
        return Internal::dataToBlock<Types::Line2D>(data);
    case Line3D:
        return Internal::dataToBlock<Types::Line3D>(data);
    case Plane3D:
        return Internal::dataToBlock<Types::Plane3D>(data);
    case XYT:
        return Internal::dataToBlock<Types::XYT>(data);
    case XYZT:
        return Internal::dataToBlock<Types::XYZT>(data);
    case PRIOR:
        return Internal::dataToBlock<Types::PRIOR>(data);
    case PRIORID:
        return Internal::dataToBlock<Types::PRIORID>(data);
    case INVALID:
        break;
    };

    return Typed<Types::INVALID>();

}
QString GenericToString(Generic const& corresp) {
    return std::visit([](auto const& arg){ return arg.toStr(); }, corresp);
}

std::optional<std::tuple<Eigen::Matrix<double,2,3>,Eigen::Vector2d>>
getGeoXYConstraintInfos(Typed<Types::GEOXY> const& point,
                        StereoVision::Geometry::AffineTransform<double> const& ecef2local,
                        double height) {

    Eigen::Matrix<double,2,3> M;
    Eigen::Vector2d v;

    PJ_CONTEXT* ctx = proj_context_create();

    const char* WGS84_ellipsoid = "EPSG:4979";
    const char* WGS84_ecef = "EPSG:4978";

    PJ* ellipsoidconverter = proj_create_crs_to_crs(ctx, point.crsInfos.toStdString().c_str(), WGS84_ellipsoid, nullptr);

    if (ellipsoidconverter == 0) { //in case of error
        proj_context_destroy(ctx);
        return std::nullopt;
    }

    PJ* ecefconverter = proj_create_crs_to_crs(ctx, WGS84_ellipsoid, WGS84_ecef, nullptr);

    if (ecefconverter == 0) { //in case of error
        proj_destroy(ellipsoidconverter);
        proj_context_destroy(ctx);
        return std::nullopt;
    }

    PJ_COORD coord;
    coord.xy.x = point.x;
    coord.xy.y = point.y;

    PJ_COORD ellipsoid1 = proj_trans(ellipsoidconverter, PJ_FWD, coord);
    PJ_COORD ellipsoid2 = ellipsoid1;

    ellipsoid1.lpz.z = 0;
    ellipsoid2.lpz.z = height;

    PJ_COORD ptLow = proj_trans(ecefconverter, PJ_FWD, ellipsoid1);
    PJ_COORD ptHigh = proj_trans(ecefconverter, PJ_FWD, ellipsoid2);

    Eigen::Vector3d pLow;
    pLow.x() = ptLow.xyz.x;
    pLow.y() = ptLow.xyz.y;
    pLow.z() = ptLow.xyz.z;

    Eigen::Vector3d pHigh;
    pHigh.x() = ptHigh.xyz.x;
    pHigh.y() = ptHigh.xyz.y;
    pHigh.z() = ptHigh.xyz.z;

    proj_destroy(ellipsoidconverter);
    proj_destroy(ecefconverter);
    proj_context_destroy(ctx);

    Eigen::Vector3d pLowLocal = ecef2local*pLow;
    Eigen::Vector3d pHighLocal = ecef2local*pHigh;

    Eigen::Vector3d normalVec = pHighLocal - pLowLocal;

    if (normalVec.norm() < 1e-5*height) {
        return std::nullopt; //could not get a normal vector
    }

    int maxId = (std::abs(normalVec[0]) >= std::abs(normalVec[1]) and std::abs(normalVec[0]) >= std::abs(normalVec[2])) ?
                0 : (std::abs(normalVec[1]) >= std::abs(normalVec[2])) ? 1 : 2;

    normalVec.normalize();

    Eigen::Vector3d perp1;
    perp1[maxId] = -(normalVec[(maxId+1)%3] + normalVec[(maxId+2)%3]);
    perp1[(maxId+1)%3] = normalVec[maxId];
    perp1[(maxId+2)%3] = normalVec[maxId];

    perp1.normalize();

    Eigen::Vector3d perp2 = perp1.cross(normalVec);
    perp2.normalize();

    M.row(0) = perp1;
    M.row(1) = perp2;

    v = M*pLowLocal;

    return std::make_tuple(M,v);

}


std::optional<Eigen::Vector3d> getGeoXYZConstraintInfos(Typed<Types::GEOXYZ> const& point,
                                         StereoVision::Geometry::AffineTransform<double> const& ecef2local) {

    Eigen::Vector3d ret;

    PJ_CONTEXT* ctx = proj_context_create();

    PJ* converter = proj_create_crs_to_crs(ctx, point.crsInfos.toStdString().c_str(), "EPSG:4978", nullptr);

    if (converter == 0) { //in case of error
        proj_context_destroy(ctx);
        return std::nullopt;
    }

    PJ_COORD coord;
    coord.xyz.x = point.x;
    coord.xyz.y = point.y;
    coord.xyz.z = point.z;

    PJ_COORD converted = proj_trans(converter, PJ_FWD, coord);

    ret.x() = converted.xyz.x;
    ret.y() = converted.xyz.y;
    ret.z() = converted.xyz.z;

    proj_destroy(converter);
    proj_context_destroy(ctx);

    return ecef2local*ret;

}

} // namespace Correspondences

} // namespace StereoVisionApp
