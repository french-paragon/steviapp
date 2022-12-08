#include "pointcloudio.h"

#include <cmath>

#include <type_traits>

#include <QFile>
#include <QTextStream>

struct Point {
	float x;
	float y;
	float z;
	uint8_t r;
	uint8_t g;
	uint8_t b;
};

struct Cloud {
	uint32_t width;
	uint32_t height;
	bool is_dense;
	std::vector<Point> points;
};

int saveCloud(QString const& filename, Cloud const& cloud) {

	if (!filename.endsWith(".pcd")) {
		return -1;
	}

	QFile file(filename);

	if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
	{
		return -1;
	}

	QTextStream stream(&file);

	stream << "VERSION .7\n";
	stream << "FIELDS x y z rgb" << "\n";
	stream << "SIZE 4 4 4 4" << "\n";
	stream << "TYPE F F F U" << "\n";
	stream << "COUNT 1 1 1 1" << "\n";
	stream << "WIDTH" << cloud.width << "\n";
	stream << "HEIGHT" << cloud.height << "\n";
	stream << "VIEWPOINT 0 0 0 1 0 0 0" << "\n";
	stream << "POINTS " << cloud.points.size() << "\n";
	stream << "DATA ascii";

	for (Point const& p : cloud.points) {
		uint32_t color = 0;

		color += p.r;
		color <<= 8;

		color += p.g;
		color <<= 8;

		color += p.b;

		stream << "\n" << p.x << ' ' << p.y << ' ' << p.z << ' ' << color;
	}

	file.close();

	return 0;
}

namespace StereoVisionApp {

int saveXYZRGBpointcloud(QString file, Multidim::Array<float, 3> const& geom, Multidim::Array<float, 3> const& color, float colorScale, bool correctGamma) {

	if (file.isEmpty()) {
		return -1;
	}

	constexpr Multidim::AccessCheck Nc = Multidim::AccessCheck::Nocheck;

	auto gShape = geom.shape();
	auto cShape = color.shape();

	if (gShape != cShape) {
		return -1;
	}

	if (gShape[2] != 3) {
		return -1;
	}

	size_t h = gShape[0];
	size_t w = gShape[1];

	// Fill in the cloud data
	Cloud cloud;

	cloud.width = static_cast<uint32_t>(w);
	cloud.height = static_cast<uint32_t>(h);
	cloud.is_dense = false;
	cloud.points.resize (cloud.width * cloud.height);

	for (size_t i = 0; i < h; ++i) {
		for (size_t j = 0; j < w; ++j) {

			cloud.points[i*w + j].x = geom.value<Nc>(i,j,0);
			cloud.points[i*w + j].y = geom.value<Nc>(i,j,1);
			cloud.points[i*w + j].z = geom.value<Nc>(i,j,2);

			float r = color.value<Nc>(i,j,0);
			float g = color.value<Nc>(i,j,1);
			float b = color.value<Nc>(i,j,2);

			if (std::isnan(r) or std::isnan(g) or std::isnan(b)) {
				cloud.points[i*w + j].x = std::nanf("");
				cloud.points[i*w + j].y = std::nanf("");
				cloud.points[i*w + j].z = std::nanf("");
			}

			if (correctGamma) {
				r = std::pow<float>(r, float(1/2.2));
				g = std::pow<float>(g, float(1/2.2));
				b = std::pow<float>(b, float(1/2.2));
			}

			r = (r < 0) ? 0 : r;
			g = (g < 0) ? 0 : g;
			b = (b < 0) ? 0 : b;

			cloud.points[i*w + j].r = static_cast<uint8_t>(std::roundf(colorScale*r));
			cloud.points[i*w + j].g = static_cast<uint8_t>(std::roundf(colorScale*g));
			cloud.points[i*w + j].b = static_cast<uint8_t>(std::roundf(colorScale*b));

		}
	}

	QString out = file;

	if (!file.endsWith(".pcd")) {
		out += ".pcd";
	}

	int code = -1;

	if (file.endsWith(".pcd")) {

		code = saveCloud(file, cloud);
		//code = pcl::io::savePCDFileBinary(file.toStdString(), cloud);

	}

	return code;
}

} // namespace StereoVisionApp
