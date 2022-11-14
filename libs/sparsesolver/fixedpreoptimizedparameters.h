#ifndef STEREOVISIONAPP_FIXEDPREOPTIMIZEDPARAMTERS_H
#define STEREOVISIONAPP_FIXEDPREOPTIMIZEDPARAMTERS_H

#include <QFlags>

namespace StereoVisionApp {

enum FixedParameter {
	NoFixedParameters = 0,
	CameraInternal = 1,
	StereoRigs = 2,
	CameraExternal = 4
};
Q_DECLARE_FLAGS(FixedParameters, FixedParameter);
Q_DECLARE_OPERATORS_FOR_FLAGS(FixedParameters);

}

#endif // STEREOVISIONAPP_FIXEDPREOPTIMIZEDPARAMTERS_H
