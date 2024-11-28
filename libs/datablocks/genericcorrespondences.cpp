#include "genericcorrespondences.h"

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
        if (idxNSpace < 0 and !str[0].isSpace()) {
            idxNSpace = i;
        } else {
            if (str[0].isSpace()) {
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
    case INVALID:
        break;
    };

    return Typed<Types::INVALID>();

}
QString GenericToString(Generic const& corresp) {
    return std::visit([](auto const& arg){ return arg.toStr(); }, corresp);
}

} // GenericCorrespondences

} // namespace StereoVisionApp
