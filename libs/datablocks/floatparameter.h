#ifndef STEREOVISIONAPP_FLOATPARAMETER_H
#define STEREOVISIONAPP_FLOATPARAMETER_H

#include <QMetaType>

#include <QJsonObject>
#include <QJsonValue>
#include <QJsonArray>

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
    bool operator!= (floatParameter const& other) const;
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

    inline pFloatType valueOr(pFloatType const& alt) {
        return (_isSet) ? _value : alt;
    }

	bool isUncertain() const;
	void setUncertainty();
	void setUncertainty(pFloatType stddev);
	void clearUncertainty();

	pFloatType & stddev();
	pFloatType const& stddev() const;

	static QJsonObject toJson(floatParameter const& fP);
	static floatParameter fromJson(QJsonObject const& obj);
protected:

	bool _isSet;
	bool _isUncertain;
	pFloatType _value;
	pFloatType _stddev;

private:
	static int registrationCode;
};

template<int size>
class floatParameterGroup
{
	static_assert (size >= 2, "a float parameter group should have size at least bigger than two.");

	static const int CovMatSize = size*(size+1)/2;
public:
	floatParameterGroup() :
		_isSet(false),
		_isUncertain(false)
	{
		_parameters.fill(0);
		_CovMatTrig.fill(0);
	}
	floatParameterGroup(floatParameterGroup const& other) :
		_parameters(other._parameters),
		_CovMatTrig(other._CovMatTrig),
		_isSet(other._isSet),
		_isUncertain(other._isUncertain)
	{

	}


	floatParameterGroup& operator= (floatParameterGroup const& other) {
		_parameters = other._parameters;
		_CovMatTrig = other._CovMatTrig;
		_isSet = other._isSet;
		_isUncertain = other._isUncertain;
		return *this;
	}

	bool operator== (floatParameterGroup const& other) const {
		bool c = _isSet and other._isSet;
		if (_isSet) {
			c = c and _parameters == other._parameters;
		}

		c = c and _isUncertain and other.isUncertain();
		if (_isUncertain) {
			c = c and _CovMatTrig == other._CovMatTrig;
		}

		return c;
	}

    bool operator!= (floatParameterGroup const& other) const {
        bool c = _isSet != other._isSet;
        if (_isSet) {
            c = c or _parameters != other._parameters;
        }

        c = c or _isUncertain != other.isUncertain();
        if (_isUncertain) {
            c = c or _CovMatTrig != other._CovMatTrig;
        }

        return c;
    }

	bool isApproximatlyEqual (floatParameterGroup const& other, pFloatType tol = 1e-4) const {
		bool c = _isSet == other._isSet;
		if (_isSet) {
			for (int i = 0; i < size; i++) {
				c = c and std::abs((_parameters[i] - other._parameters[i])/_parameters[i]) < tol;
			}
		}

		c = c and _isUncertain == other.isUncertain();
		if (_isUncertain) {
			for (int i = 0; i < CovMatSize; i++) {
				c = c and std::abs((_CovMatTrig[i] - other._CovMatTrig[i])/_CovMatTrig[i]) < tol;
			}
		}

		return c;
	}

	bool isSet() const {
		return _isSet;
	}
	void setIsSet() {
		_isSet = true;
	}
	void clearIsSet() {
		_isSet = false;
	}

	pFloatType& value(int index) {
		return _parameters[index];
	}
	pFloatType const& value(int index) const {
		return _parameters[index];
	}

	bool isUncertain() const {
		return _isUncertain;
	}
	void setUncertain() {
		_isUncertain = true;
	}
	void clearUncertain() {
		_isUncertain = false;
	}

	pFloatType & stddev(int index1, int index2 = -1) {
		int id1 = index1;
		int id2 = (index2 >= 0) ? index2 : index1;

		if (id1 < id2) {
			int tmp = id2;
			id2 = id1;
			id1 = tmp;
		}

		int id = id1 - id2;
		id += size*(id2);
		id -= (id2)*(id2-1)/2;

		return _CovMatTrig[id];
	}
	pFloatType const& stddev(int index1, int index2 = -1) const {
		int id1 = index1;
		int id2 = (index2 >= 0) ? index2 : index1;

		if (id1 < id2) {
			int tmp = id2;
			id2 = id1;
			id1 = tmp;
		}

		int id = id1 - id2;
		id += size*(id2);
		id -= (id2)*(id2-1)/2;

		return _CovMatTrig[id];
	}

	static QJsonObject toJson(floatParameterGroup const& fpg) {
		QJsonObject obj;

		obj.insert("isSet", static_cast<int>(fpg.isSet()));

		QJsonArray vals;
		for (int i = 0; i < size; i++) {
			vals.push_back(static_cast<qreal>(fpg.value(i)));
		}

		obj.insert("vals", vals);

		obj.insert("isUncertain", static_cast<int>(fpg.isUncertain()));


		QJsonArray covMat;
		for (int i = 0; i < CovMatSize; i++) {
			vals.push_back(static_cast<qreal>(fpg._CovMatTrig[i]));
		}

		obj.insert("CovMatTrig", covMat);

		return obj;
	}
	static floatParameterGroup fromJson(QJsonObject const& obj) {

		floatParameterGroup fpg;

		if (obj.contains("vals")) {
			QJsonValue v = obj.value("vals");
			QJsonArray vals = v.toArray();

			if (vals.size() == size) {
				for (int i = 0; i < size; i++) {
					fpg._parameters[i] = static_cast<pFloatType>(vals.at(i).toDouble());
				}

				if (obj.contains("isSet")) {
					QJsonValue v = obj.value("isSet");
					if (v.toBool() or v.toInt() > 0) {
						fpg.setIsSet();
					}
				}
			}
		}

		if (obj.contains("stddev")) {
			QJsonValue v = obj.value("stddev");
			QJsonArray vals = v.toArray();

			if (vals.size() == CovMatSize) {

				for (int i = 0; i < CovMatSize; i++) {
					fpg._CovMatTrig[i] = static_cast<pFloatType>(vals.at(i).toDouble());
				}

				if (obj.contains("isUncertain")) {
					QJsonValue v = obj.value("isUncertain");
					if (v.toBool() or v.toInt() > 0) {
						fpg.setUncertain();
					}
				}
			}

		}

		return fpg;

	}

private:
	std::array<pFloatType, size> _parameters;
	std::array<pFloatType, CovMatSize> _CovMatTrig;
	bool _isSet;
	bool _isUncertain;
};

} // namespace StereoVisionApp

Q_DECLARE_METATYPE(StereoVisionApp::floatParameter);
Q_DECLARE_METATYPE(StereoVisionApp::floatParameterGroup<2>);
Q_DECLARE_METATYPE(StereoVisionApp::floatParameterGroup<3>);
Q_DECLARE_METATYPE(StereoVisionApp::floatParameterGroup<4>);
Q_DECLARE_METATYPE(StereoVisionApp::floatParameterGroup<5>);
Q_DECLARE_METATYPE(StereoVisionApp::floatParameterGroup<6>);

#endif // STEREOVISIONAPP_FLOATPARAMETER_H
