#include "imagedatadisplayadapter.h"

namespace StereoVisionApp {

ImageDataDisplayAdapter::ImageDataDisplayAdapter(QObject *parent) :
    _loaded_image(),
    _gradient(std::nullopt)
{

}

QSize ImageDataDisplayAdapter::getImageSize() const {
    return _loaded_image.size();
}

QColor ImageDataDisplayAdapter::getColorAtPoint(int x, int y) const {
    return _loaded_image.pixelColor(x,y);
}

QImage ImageDataDisplayAdapter::getImage() const {
    return _loaded_image;
}

void ImageDataDisplayAdapter::setGradient(std::function<QColor(float)> const& newGradient)
{
    _gradient = newGradient;
}

QVector<ImageDataDisplayAdapter::ChannelInfo> ImageDataDisplayAdapter::getOriginalChannelsInfos(QPoint const& pos) const {

    int i = pos.y();
    int j = pos.x();

    QVector<ChannelInfo> ret;

    for (int c = 0; c < _original_data.shape()[2]; c++) {
        QString channelName = QString("C%1").arg(c+1);
        QString channelVal = QString("%1").arg(_original_data.valueOrAlt({i,j,c}, std::nanf("")));
        ret.push_back({channelName, channelVal});
    }

    return ret;
}

} // namespace StereoVisionApp
