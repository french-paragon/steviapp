#ifndef STEREOVISIONAPP_COLLADA_EXPORT_FILTER_H
#define STEREOVISIONAPP_COLLADA_EXPORT_FILTER_H

#include <QTextStream>

namespace StereoVisionApp {

class Project;

void exportOptimizedToCollada(QTextStream & oStream, Project* p, bool optimizedOnly);

}

#endif // STEREOVISIONAPP_COLLADA_EXPORT_FILTER_H
