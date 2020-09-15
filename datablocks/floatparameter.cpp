#include "floatparameter.h"

#include <cmath>

int StereoVisionApp::floatParameter::registrationCode = qRegisterMetaType<StereoVisionApp::floatParameter>();

namespace StereoVisionApp {

floatParameter::floatParameter() :
	_isSet(false),
	_isUncertain(false),
	_value(0.0),
	_stddev(0.0)
{

}
floatParameter::floatParameter(pFloatType value, bool isSet) :
	_isSet(isSet),
	_isUncertain(false),
	_value(value),
	_stddev(0.0)
{

}
floatParameter::floatParameter(pFloatType value, pFloatType stddev) :
	_isSet(true),
	_isUncertain(true),
	_value(value),
	_stddev(stddev)
{

}
floatParameter::floatParameter(floatParameter const& other) :
	_isSet(other.isSet()),
	_isUncertain(other.isUncertain()),
	_value(other.value()),
	_stddev(other.stddev())
{

}

bool floatParameter::operator< (floatParameter const& other) const {
	return _value < other.value();
}
bool floatParameter::operator== (floatParameter const& other) const {
	return (other._isSet == _isSet) and (other._isUncertain == _isUncertain) and (other._value == _value) and (other._stddev == _stddev);
}
bool floatParameter::isApproximatlyEqual (floatParameter const& other, pFloatType tol) const {
	if (_isSet and other.isSet()) {
		return std::abs(_value - other.value()) < tol;
	} else {
		return false;
	}
}

floatParameter floatParameter::operator+ (floatParameter const& other) const {

	floatParameter r(other);
	r += *this;
	return r;
}
floatParameter floatParameter::operator+ (pFloatType const& val) const {

	floatParameter r(*this);
	r += val;
	return r;
}
floatParameter floatParameter::operator- () const {

	floatParameter r(*this);
	r.value() = -r.value();
	return r;
}
floatParameter floatParameter::operator- (floatParameter const& other) const {

	return *this + (-other);
}

floatParameter floatParameter::operator- (pFloatType const& val) const {
	return *this + (-val);
}

floatParameter floatParameter::operator* (floatParameter const& other) const {

	floatParameter r(*this);
	r *= other;
	return r;
}
floatParameter floatParameter::operator* (pFloatType const& val) const {

	floatParameter r(*this);
	r += val;
	return r;
}
floatParameter floatParameter::operator/ (floatParameter const& other) const {

	floatParameter r(*this);
	r /= other;
	return r;
}
floatParameter floatParameter::operator/ (pFloatType const& val) const {

	floatParameter r(*this);
	r += val;
	return r;
}

floatParameter& floatParameter::operator= (floatParameter const& other) {

	_isSet = other.isSet();
	_isUncertain = other.isUncertain();
	_value = other.value();
	_stddev = other.stddev();

	return *this;
}
floatParameter& floatParameter::operator+= (floatParameter const& other) {

	if (other.isSet()) {
		_value += other.value();

		if (other.isUncertain()) {
			setUncertainty();
			_stddev += other._stddev;
		}
	}

	return *this;
}
floatParameter& floatParameter::operator-= (floatParameter const& other) {

	if (other.isSet()) {
		_value -= other.value();

		if (other.isUncertain()) {
			setUncertainty();
			_stddev += other._stddev;
		}
	}

	return *this;
}
floatParameter& floatParameter::operator*= (floatParameter const& other) {

	if (other.isSet()) {

		if (other.isUncertain()) {
			setUncertainty();
			_stddev = std::sqrt(other._stddev*other._stddev*_value*_value + _stddev*_stddev*other._value*other._value);
		} else if (isUncertain()) {
			_stddev = _stddev*other._value;
		}
		_value *= other.value();
	}

	return *this;
}
floatParameter& floatParameter::operator/= (floatParameter const& other) {

	if (other.isSet()) {

		if (other.isUncertain()) {
			setUncertainty();
			pFloatType tmp = other._value*other._value;
			_stddev = std::sqrt(other._stddev*other._stddev*_value*_value/(tmp*tmp) + _stddev*_stddev/tmp);
		} else if (isUncertain()) {
			pFloatType tmp = other._value*other._value;
			_stddev = _stddev/tmp;
		}
		_value /= other.value();
	}

	return *this;
}

floatParameter& floatParameter::operator= (pFloatType const& val) {
	_value = val;
	return *this;
}
floatParameter& floatParameter::operator+= (pFloatType const& val) {
	_value += val;
	return *this;
}
floatParameter& floatParameter::operator-= (pFloatType const& val) {
	_value -= val;
	return *this;}
floatParameter& floatParameter::operator*= (pFloatType const& val) {

	_value *= val;
	_stddev *= std::abs(val);
	return *this;
}
floatParameter& floatParameter::operator/= (pFloatType const& val) {

	_value /= val;
	_stddev /= std::abs(val);
	return *this;

}

bool floatParameter::isSet() const {
	return _isSet;
}
void floatParameter::setIsSet() {
	_isSet = true;
}
void floatParameter::setIsSet(pFloatType value) {
	_value = value;
	_isSet = true;
}
void floatParameter::clearIsSet() {
	_isSet = false;
}

pFloatType& floatParameter::value() {
	return _value;
}
pFloatType const& floatParameter::value() const {
	return _value;
}

bool floatParameter::isUncertain() const {
	return _isUncertain;
}
void floatParameter::setUncertainty() {
	_isUncertain = true;
}
void floatParameter::setUncertainty(pFloatType stddev) {
	_stddev = stddev;
	_isUncertain = true;
}
void floatParameter::clearUncertainty() {
	_isUncertain = false;
}

pFloatType& floatParameter::stddev() {
	return _stddev;
}
pFloatType const& floatParameter::stddev() const {
	return _stddev;
}

} // namespace StereoVisionApp
