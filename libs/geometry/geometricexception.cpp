#include "geometricexception.h"

namespace StereoVisionApp {

GeometricException::GeometricException(QString const& what)
{
	std::string w = what.toStdString();
	_what = new char[w.size()];
	memcpy (_what, w.c_str(), w.size());
}


GeometricException::GeometricException(GeometricException const& other) :
	GeometricException(other.what())
{

}

GeometricException::~GeometricException() {
	delete _what;
}

const char* GeometricException::what() const noexcept
{
	return _what;
}

} // namespace StereoVisionApp
