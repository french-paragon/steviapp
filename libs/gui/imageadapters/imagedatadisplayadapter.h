#ifndef STEREOVISIONAPP_IMAGEDATADISPLAYADAPTER_H
#define STEREOVISIONAPP_IMAGEDATADISPLAYADAPTER_H

#include <qImageDisplayWidget/imageadapter.h>

#include <MultidimArrays/MultidimArrays.h>

#include <LibStevi/utils/types_manipulations.h>

#include <cmath>
#include <optional>
#include <functional>

namespace StereoVisionApp {

class ImageDataDisplayAdapter : public QImageDisplay::ImageAdapter
{
    Q_OBJECT
public:
    ImageDataDisplayAdapter(QObject *parent = nullptr);

    template<typename T>
    void setImageFromArray(Multidim::Array<T,3> const& arr, T whiteLevel, T blackLevel, bool logConvert = false) {

        T wLevel = whiteLevel;
        T bLevel = blackLevel;

        if (logConvert) {
            wLevel = std::log(wLevel+1);
            bLevel = std::log(bLevel+1);
        }

        QSize oldSize = _loaded_image.size();

        _loaded_image = QImage(arr.shape()[1], arr.shape()[0], QImage::Format_RGB888);
        _original_data = Multidim::Array<float,3>(arr.shape()[0], arr.shape()[1], 3);

        for (int i = 0; i < arr.shape()[0]; i++) {
            for (int j = 0; j < arr.shape()[1]; j++) {

                std::array<uint8_t, 3> color;

                for (int c = 0; c < 3; c++) {
                    T val = arr.valueOrAlt({i,j,c}, 0);
                    _original_data.atUnchecked(i,j,c) = val;

                    if (logConvert) {
                        val = std::log(val+1);
                    }

                    color[c] = valueToColor(val, bLevel, wLevel);
                }

                QColor col;

                col.setRed(color[0]);
                col.setGreen(color[1]);
                col.setBlue(color[2]);

                _loaded_image.setPixelColor(j,i,col);
            }
        }

        if (_loaded_image.size() != oldSize) {
            Q_EMIT imageSizeChanged(_loaded_image.size());
        }

        Q_EMIT imageValuesChanged(QRect(QPoint(0,0), _loaded_image.size()));
    }

    template<typename T>
    void setImageFromArray(Multidim::Array<T,2> const& arr, T whiteLevel, T blackLevel, bool logConvert = false) {

        T wLevel = whiteLevel;
        T bLevel = blackLevel;

        if (logConvert) {
            wLevel = std::log(wLevel+1);
            bLevel = std::log(bLevel+1);
        }

        QSize oldSize = _loaded_image.size();

        _loaded_image = QImage(arr.shape()[1], arr.shape()[0], QImage::Format_RGB888);
        _original_data = Multidim::Array<float,3>(arr.shape()[0], arr.shape()[1], 1);

        for (int i = 0; i < arr.shape()[0]; i++) {
            for (int j = 0; j < arr.shape()[1]; j++) {

                uint8_t color;
                T val = arr.valueUnchecked(i,j);

                _original_data.atUnchecked(i,j,0) = val;

                if (logConvert) {
                    val = std::log(val+1);
                }

                QColor col;

                if (_gradient.has_value()) {
                    float prop = (static_cast<float>(val) - static_cast<float>(bLevel))
                            /(wLevel - bLevel);

                    col = _gradient.value()(prop);

                } else {

                    color = valueToColor(val, bLevel, wLevel);

                    col.setRed(color);
                    col.setGreen(color);
                    col.setBlue(color);
                }

                _loaded_image.setPixelColor(j,i,col);
            }
        }

        if (_loaded_image.size() != oldSize) {
            Q_EMIT imageSizeChanged(_loaded_image.size());
        }

        Q_EMIT imageValuesChanged(QRect(QPoint(0,0), _loaded_image.size()));

    }

    QSize getImageSize() const override;

    QColor getColorAtPoint(int x, int y) const override;

    QImage getImage() const override;

    void setGradient(std::function<QColor(float)> const& newGradient);

    QVector<ChannelInfo> getOriginalChannelsInfos(QPoint const& pos) const override;

protected:

    template<typename T>
    inline uint8_t valueToColor(T const& value, T blackLevel, T whiteLevel) const {

        using ComputeType = StereoVision::TypesManipulations::accumulation_extended_t<T>;

        if (value < blackLevel) {
            return 0;
        }

        if (value >= whiteLevel) {
            return 255;
        }

        ComputeType transformed = (255*(static_cast<ComputeType>(value) - static_cast<ComputeType>(blackLevel)))
                /(whiteLevel - blackLevel);

        return static_cast<uint8_t>(transformed);
    }

    QImage _loaded_image;
    Multidim::Array<float, 3> _original_data;

    std::optional<std::function<QColor(float)>> _gradient;

};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_IMAGEDATADISPLAYADAPTER_H
