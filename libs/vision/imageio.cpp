#include "imageio.h"

#include <QImage>
#include <cmath>

namespace StereoVisionApp {

ImageArray getImageData(QString const& imFile, float gamma, int *originalFormat) {

	QImage img(imFile);

	if (img.isNull()) {
		return ImageArray(0,0,0);
	}

	if (originalFormat != nullptr) {
		*originalFormat = img.format();
	}

	if (img.format() == QImage::Format_RGB32 or
			img.format() == QImage::Format_BGR888 or
			img.format() == QImage::Format_ARGB32 or
			img.format() == QImage::Format_RGBA8888) {

		img = img.convertToFormat(QImage::Format_RGB888);
	}

	if (img.format() == QImage::Format_RGB888) {

		ImageArray out(img.height(),img.width(),3);

		for (int i = 0; i < img.height(); i++) {
			uchar* line = img.scanLine(i);

			for(int j = 0; j < img.width(); j++) {
				float r = powf(static_cast<float>(line[j*3]) / 255.f, gamma);
				float g = powf(static_cast<float>(line[j*3+1]) / 255.f, gamma);
				float b = powf(static_cast<float>(line[j*3+2]) / 255.f, gamma);

				out.at(i,j,0) = r;
				out.at(i,j,1) = g;
				out.at(i,j,2) = b;
			}
		}

		return out;
	}

	if (img.format() == QImage::Format_Grayscale8) {

		ImageArray out(img.height(),img.width(),1);

		for (int i = 0; i < img.height(); i++) {
			uchar* line = img.scanLine(i);

			for(int j = 0; j < img.width(); j++) {
				float g = powf(static_cast<float>(line[j]) / 255.f, gamma);

				out.at(i,j,0) = g;
			}
		}

		return out;
	}

	if (img.format() == QImage::Format_Grayscale16) {

		ImageArray out(img.height(),img.width(),1);

		for (int i = 0; i < img.height(); i++) {
			quint16* line = reinterpret_cast<quint16*>(img.bits() + i * img.bytesPerLine());

			for(int j = 0; j < img.width(); j++) {
				float g = powf(static_cast<float>(line[j]) / 65535.f, gamma);

				out.at(i,j,0) = g;
			}
		}

		return out;
	}

	return ImageArray(0,0,0);
}

uchar floatToBitColor(float f, float gamma) {

	if (f < 0) {
		return 0;
	}

	float corrected = powf(f, 1/gamma);
	corrected *= 256.;
	if (corrected >= 255) return 255;

	return static_cast<uchar>(std::floor(corrected));
}

uint16_t floatToUInt16Color(float f, float gamma) {

	if (f < 0) {
		return 0;
	}

	float corrected = powf(f, 1/gamma);
	corrected *= 65536.;
	if (corrected >= 65535) return 65535;

	return static_cast<uint16_t>(std::floor(corrected));
}

bool saveImageData(QString const& imFile,
				   ImageArray const& imageData,
				   float gamma,
				   std::optional<int> imgFormat)
{

	auto shape = imageData.shape();

	int& height = shape[0];
	int& width = shape[1];
	int& c = shape[2];

	if (c != 3 and c != 1) {
		return false;
	}

	QImage::Format f = (c == 3) ? QImage::Format_RGB888 : QImage::Format_Grayscale8;

	if (imgFormat.has_value()) {
		int v = imgFormat.value();

		if (c == 1 and (v == QImage::Format_Grayscale8 or v == QImage::Format_Grayscale16)) {
			f = static_cast<QImage::Format>(v);
		} else if (c == 3 and (v == QImage::Format_RGB888)) {
			f = static_cast<QImage::Format>(v);
		}
	}

	QImage img(QSize(width, height), f);

	for (int i = 0; i < img.height(); i++) {
		uchar* line = img.scanLine(i);
		quint16* line16 = reinterpret_cast<quint16*>(img.bits() + i*img.bytesPerLine());

		for(int j = 0; j < img.width(); j++) {

			if (c == 3) {

				uchar r = floatToBitColor(imageData.value(i,j,0), gamma);
				uchar g = floatToBitColor(imageData.value(i,j,1), gamma);
				uchar b = floatToBitColor(imageData.value(i,j,2), gamma);

				line[j*3] = r;
				line[j*3 + 1] = g;
				line[j*3 + 2] = b;

			} else {

				if (f == QImage::Format_Grayscale8) {
					uchar g = floatToBitColor(imageData.value(i,j,0), gamma);
					line[j] = g;
				} else if (f == QImage::Format_Grayscale16) {
					uint16_t g = floatToUInt16Color(imageData.value(i,j,0), gamma);
					line16[j] = g;
				}

			}
		}
	}

	return img.save(imFile);
}

} // namespace StereoVisionApp
