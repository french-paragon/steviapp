#include "exportactions.h"

#include "mainwindow.h"

#include "datablocks/io/collada_export_filter.h"

#include <QFileDialog>
#include <QTextStream>

namespace StereoVisionApp {

bool exportCollada(Project* p, MainWindow* w) {

	if (p == nullptr) {
		return false;
	}

	QString selfilter = QString("Collada (*.dae)");
	QString saveFile = QFileDialog::getSaveFileName(w, QObject::tr("Export to collada"), QString(), "Collada (*.dae);; All files (*.*)", &selfilter);

	if (saveFile.isEmpty()) {
		return false;
	}

	QFile f(saveFile);

	if (!f.open(QFile::WriteOnly | QFile::Truncate)) {
		return false;
	}

	QTextStream out(&f);

	exportOptimizedToCollada(out, p, true);

	f.close();

	return true;
}

} //namespace StereoVisionApp
