#include "landmarkbasedactions.h"

#include "datablocks/project.h"
#include "datablocks/landmark.h"

#include <QFile>

namespace StereoVisionApp {

void exportLandmarksToCsv(Project* p, QVector<qint64> const& landmarks, QString const& file) {

	if (p == nullptr) {
		return;
	}

	QFile f(file);

	if (!f.open(QFile::WriteOnly)) {
		return;
	}

	QTextStream s(&f);

	s << "Landmark"
	  << ", x position"
	  << ", y position"
	  << ", z position";

	for (qint64 id : landmarks) {
		Landmark* lm = p->getDataBlock<Landmark>(id);

		if (lm == nullptr) {
			continue;
		}

		if (lm->hasOptimizedParameters()) {
			s << '\n' << lm->objectName()
			  << ',' << lm->optPos().value(0)
			  << ',' << lm->optPos().value(1)
			  << ',' << lm->optPos().value(2);
		} else {
			s << '\n' << lm->objectName()
			  << ',' << ' '
			  << ',' << ' '
			  << ',' << ' ';
		}
	}

	f.close();
}

} //namespace StereoVisionApp
