#include "imagedatablockdisplayadapter.h"

#include "datablocks/image.h"

namespace StereoVisionApp {

ImageDatablockDisplayAdapter::ImageDatablockDisplayAdapter(QObject *parent):
    QImageDisplay::ImageAdapter(parent),
    _currentImageDataBlock(nullptr)
{

}

void ImageDatablockDisplayAdapter::setImageDatablock(Image* img) {


    if (_currentImageDataBlock != nullptr) {
        disconnect(_currentImageDataBlock, &QObject::destroyed, this, &ImageDatablockDisplayAdapter::imageDeleted);
        disconnect(_currentImageDataBlock, &Image::imageFileChanged, this, &ImageDatablockDisplayAdapter::reloadImage);
    }

    _currentImageDataBlock = img;

    if (img != nullptr) {
        connect(_currentImageDataBlock, &QObject::destroyed, this, &ImageDatablockDisplayAdapter::imageDeleted);
        connect(_currentImageDataBlock, &Image::imageFileChanged, this, &ImageDatablockDisplayAdapter::reloadImage);

        reloadImage();
    }
}


void ImageDatablockDisplayAdapter::imageDeleted() {
    _currentImageDataBlock = nullptr;
}



QSize ImageDatablockDisplayAdapter::getImageSize() const {
    return _loaded_image.size();
}

QColor ImageDatablockDisplayAdapter::getColorAtPoint(int x, int y) const {
    return _loaded_image.pixelColor(x, y);
}

QImage ImageDatablockDisplayAdapter::getImage() const {
    return _loaded_image;
}
void ImageDatablockDisplayAdapter::reloadImage() {

    if (_currentImageDataBlock == nullptr) {
        _loaded_image = QImage();
        Q_EMIT imageValuesChanged(QRect());
        return;
    }

    QString p = _currentImageDataBlock->getImageFile();

    if (!p.isEmpty()) {
        _loaded_image = QImage(p);
    } else {
        _loaded_image = QImage();
    }
    Q_EMIT imageValuesChanged(QRect());
}

} // namespace StereoVisionApp
