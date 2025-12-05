#ifndef TYPE_CONVERSIONS_H
#define TYPE_CONVERSIONS_H

#include <QImage>
#include <QPixmap>

#include <MultidimArrays/MultidimArrays.h>

namespace StereoVisionApp {

template<typename T>
QPixmap pixmapFromImgdata(Multidim::Array<T,3> const& imgData) {

    QImage::Format format = QImage::Format_RGB32;

    if (imgData.shape()[2] == 4) {
        format = QImage::Format_ARGB32;
    } else if (imgData.shape()[2] == 1) {
        format = QImage::Format_Grayscale8;
    }

    float blackLevel = 0;
    float whiteLevel = 1;

    if constexpr (std::is_floating_point_v<T>) {

        for (int i = 0; i < imgData.shape()[0]; i++) {
            for (int j = 0; j < imgData.shape()[1]; j++) {
                for (int c = 0; c < imgData.shape()[2]; c++) {
                    T val = imgData.valueUnchecked(i,j,c);
                    whiteLevel = std::max(whiteLevel,val);
                    blackLevel = std::min(blackLevel,val);
                }
            }
        }
    }

    QImage ret(QSize(imgData.shape()[1], imgData.shape()[0]), format);

    for (int i = 0; i < imgData.shape()[0]; i++) {
        for (int j = 0; j < imgData.shape()[1]; j++) {

            uint8_t r = 255;
            uint8_t g = 255;
            uint8_t b = 255;
            uint8_t a = 255;

            if (imgData.shape()[2] == 1) {
                T tmp = imgData.valueUnchecked(i,j,0);
                if constexpr (std::is_floating_point_v<T>) {
                    r = std::min<int>(255,256*((tmp - blackLevel)/whiteLevel));
                    g = std::min<int>(255,256*((tmp - blackLevel)/whiteLevel));
                    b = std::min<int>(255,256*((tmp - blackLevel)/whiteLevel));
                } else {
                    int shift = sizeof(T)-1;
                    r = tmp >> shift;
                    g = tmp >> shift;
                    b = tmp >> shift;
                }
            } else if (imgData.shape()[2] == 3) {
                T tmpR = imgData.valueUnchecked(i,j,0);
                T tmpB = imgData.valueUnchecked(i,j,1);
                T tmpG = imgData.valueUnchecked(i,j,2);
                if constexpr (std::is_floating_point_v<T>) {
                    r = std::min<int>(255,256*((tmpR - blackLevel)/whiteLevel));
                    g = std::min<int>(255,256*((tmpG - blackLevel)/whiteLevel));
                    b = std::min<int>(255,256*((tmpB - blackLevel)/whiteLevel));
                } else {
                    int shift = sizeof(T)-1;
                    r = tmpR >> shift;
                    g = tmpG >> shift;
                    b = tmpB >> shift;
                }
            } else if (imgData.shape()[2] == 4) {
                T tmpR = imgData.valueUnchecked(i,j,0);
                T tmpB = imgData.valueUnchecked(i,j,1);
                T tmpG = imgData.valueUnchecked(i,j,2);
                T tmpA = imgData.valueUnchecked(i,j,3);
                if constexpr (std::is_floating_point_v<T>) {
                    r = std::min<int>(255,256*((tmpR - blackLevel)/whiteLevel));
                    g = std::min<int>(255,256*((tmpG - blackLevel)/whiteLevel));
                    b = std::min<int>(255,256*((tmpB - blackLevel)/whiteLevel));
                    a = std::min<int>(255,256*((tmpA - blackLevel)/whiteLevel));
                } else {
                    int shift = sizeof(T)-1;
                    r = tmpR >> shift;
                    g = tmpG >> shift;
                    b = tmpB >> shift;
                    a = tmpA >> shift;
                }
            }
            ret.setPixelColor(QPoint(j,i), QColor(r,g,b,a));
        }
    }

    return QPixmap::fromImage(ret);
}

} // namespace StereoVisionApp

#endif // TYPE_CONVERSIONS_H
