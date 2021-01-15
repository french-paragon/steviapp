#include "imageio.h"

#include <QImage>
#include <cmath>

namespace StereoVisionApp {

ImageArray getImageData(QString const& imFile, float gamma) {

	QImage img(imFile);

	if (img.isNull()) {
		return ImageArray(0,0,0);
	}

	img = img.convertToFormat(QImage::Format_RGB888);

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

uchar floatToBitColor(float f, float gamma) {

	if (f < 0) {
		return 0;
	}

	float corrected = powf(f, 1/gamma);
	corrected *= 256.;
	if (corrected >= 255) return 255;

	return static_cast<uchar>(std::floor(corrected));
}

bool saveImageData(QString const& imFile,
				   ImageArray const& imageData,
				   float gamma)
{

	auto shape = imageData.shape();

	int& height = shape[0];
	int& width = shape[1];
	int& c = shape[2];

	if (c != 3 and c != 1) {
		return false;
	}

	QImage::Format f = (c == 3) ? QImage::Format_RGB888 : QImage::Format_Grayscale8;

	QImage img(QSize(width, height), f);

	for (int i = 0; i < img.height(); i++) {
		uchar* line = img.scanLine(i);

		for(int j = 0; j < img.width(); j++) {

			if (c == 3) {

				uchar r = floatToBitColor(imageData.value(i,j,0), gamma);
				uchar g = floatToBitColor(imageData.value(i,j,1), gamma);
				uchar b = floatToBitColor(imageData.value(i,j,2), gamma);

				line[j*3] = r;
				line[j*3 + 1] = g;
				line[j*3 + 2] = b;

			} else {

				uchar g = floatToBitColor(imageData.value(i,j,0), gamma);
				line[j] = g;

			}
		}
	}

	return img.save(imFile);
}

} // namespace StereoVisionApp
