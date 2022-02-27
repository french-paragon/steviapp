#include "vertexcameraparam.h"

namespace StereoVisionApp {

const int CameraInnerVertexCollection::VerticesPerCam = 4;

VertexCameraParam::VertexCameraParam()
{

}

bool VertexCameraParam::read(std::istream& is) {
	is >> _estimate;
	return true;
}

bool VertexCameraParam::write(std::ostream& os) const {
	os << _estimate << ' ';
	return os.good();
}

void VertexCameraParam::setToOriginImpl() {
	_estimate = CamParam();
}

void VertexCameraParam::oplusImpl(const double* update_) {
	_estimate += update_;
}

VertexCameraFocal::VertexCameraFocal() {

}

bool VertexCameraFocal::read(std::istream& is) {
	is >> _estimate;
	return true;
}

bool VertexCameraFocal::write(std::ostream& os) const {
	os << _estimate << ' ';
	return os.good();
}

void VertexCameraFocal::setToOriginImpl() {
	_estimate = 1.;
}

void VertexCameraFocal::oplusImpl(const double* update_) {
	_estimate += update_[0];
}

void VertexCameraFocal::setPP(Eigen::Vector2d pp) {
	_pp = pp;
}
void VertexCameraFocal::clearPP() {
	_pp = std::nullopt;
}
void VertexCameraFocal::setExtend(Eigen::Vector2d ext) {
	_extend = ext;
}
void VertexCameraFocal::clearExtend() {
	_extend = std::nullopt;
}
std::optional<CamParam> VertexCameraFocal::getCamParam() const {
	if (_pp.has_value()) {
		CamParam p;
		p.setF(_estimate);
		p.setPp(_pp.value());
		if (_extend.has_value()) {
			p.setExtend(_extend.value());
		}
		return p;
	}
	return std::nullopt;
}

} // namespace StereoVisionApp
