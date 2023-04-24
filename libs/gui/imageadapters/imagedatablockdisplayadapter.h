#ifndef STEREOVISIONAPP_IMAGEDATABLOCKDISPLAYADAPTER_H
#define STEREOVISIONAPP_IMAGEDATABLOCKDISPLAYADAPTER_H

#include <qImageDisplayWidget/imageadapter.h>

namespace StereoVisionApp {

class Image;

class ImageDatablockDisplayAdapter : public QImageDisplay::ImageAdapter
{
    Q_OBJECT
public:
    ImageDatablockDisplayAdapter(QObject *parent = nullptr);

    inline bool hasImageDatablock() const {
        return _currentImageDataBlock != nullptr;
    }

    inline Image* currentImageDatablock() const {
        return _currentImageDataBlock;
    }
    void setImageDatablock(Image* img);

    QSize getImageSize() const override;

    QColor getColorAtPoint(int x, int y) const override;

    QImage getImage() const override;

protected:

    void imageDeleted();
    void reloadImage();

    Image* _currentImageDataBlock;
    QImage _loaded_image;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_IMAGEDATABLOCKDISPLAYADAPTER_H
