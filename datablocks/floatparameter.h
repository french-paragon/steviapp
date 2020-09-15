#ifndef STEREOVISIONAPP_FLOATPARAMETER_H
#define STEREOVISIONAPP_FLOATPARAMETER_H

#include <QMetaType>

typedef float pFloatType;

namespace StereoVisionApp {

class floatParameter
{
public:
	floatParameter();
	floatParameter(pFloatType value, bool isSet=true);
	floatParameter(pFloatType value, pFloatType stddev);
	floatParameter(floatParameter const& other);

	bool operator< (floatParameter const& other) const;
	bool operator== (floatParameter const& other) const;
	bool isApproximatlyEqual (floatParameter const& other, pFloatType tol = 1e-4) const;

	floatParameter operator+ (floatParameter const& other) const;
	floatParameter operator+ (pFloatType const& val) const;
	floatParameter operator- () const;
	floatParameter operator- (floatParameter const& other) const;
	floatParameter operator- (pFloatType const& other) const;

	floatParameter operator* (floatParameter const& other) const;
	floatParameter operator* (pFloatType const& val) const;
	floatParameter operator/ (floatParameter const& other) const;
	floatParameter operator/ (pFloatType const& val) const;

	floatParameter& operator= (floatParameter const& other);
	floatParameter& operator+= (floatParameter const& other);
	floatParameter& operator-= (floatParameter const& other);
	floatParameter& operator*= (floatParameter const& other);
	floatParameter& operator/= (floatParameter const& other);

	floatParameter& operator= (pFloatType const& other);
	floatParameter& operator+= (pFloatType const& other);
	floatParameter& operator-= (pFloatType const& other);
	floatParameter& operator*= (pFloatType const& other);
	floatParameter& operator/= (pFloatType const& other);

	bool isSet() const;
	void setIsSet();
	void setIsSet(pFloatType value);
	void clearIsSet();

	pFloatType& value();
	pFloatType const& value() const;

	bool isUncertain() const;
	void setUncertainty();
	void setUncertainty(pFloatType stddev);
	void clearUncertainty();

	pFloatType & stddev();
	pFloatType const& stddev() const;

protected:

	bool _isSet;
	bool _isUncertain;
	pFloatType _value;
	pFloatType _stddev;

private:
	static int registrationCode;
};

} // namespace StereoVisionApp

Q_DECLARE_METATYPE(StereoVisionApp::floatParameter);

#endif // STEREOVISIONAPP_FLOATPARAMETER_H
