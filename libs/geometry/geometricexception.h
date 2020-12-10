#ifndef STEREOVISIONAPP_GEOMETRICEXCEPTION_H
#define STEREOVISIONAPP_GEOMETRICEXCEPTION_H

#include <QException>

namespace StereoVisionApp {

class GeometricException : public QException
{
public:
	GeometricException(QString const& what);
	GeometricException(GeometricException const& other);

	virtual ~GeometricException();

	void raise() const override { throw *this; }
	GeometricException *clone() const override { return new GeometricException(*this); }

	const char* what() const noexcept override;

protected:

	char* _what;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_GEOMETRICEXCEPTION_H
