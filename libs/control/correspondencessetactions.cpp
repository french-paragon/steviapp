#include "correspondencessetactions.h"

#include <QFileDialog>
#include <QFile>
#include <QTextStream>

#include "control/mainwindow.h"

#include "datablocks/correspondencesset.h"
#include "datablocks/genericcorrespondences.h"

namespace StereoVisionApp {

bool importCorrespondancesFromTxt(Project* project, qint64 correspSetId, QString const& importFilePath) {

    if (project == nullptr) {
        return false;
    }

    CorrespondencesSet* correspSet = project->getDataBlock<CorrespondencesSet>(correspSetId);

    if (correspSet == nullptr) {
        qint64 id = project->createDataBlock(CorrespondencesSet::staticMetaObject.className());

        if (id < 0) {
            return false;
        }

        correspSet = project->getDataBlock<CorrespondencesSet>(id);

        if (correspSet == nullptr) {
            return false;
        }

        correspSet->setObjectName("Imported correspondence set");
    }

    QString inFile = importFilePath;

    if (inFile.isEmpty()) {

        MainWindow* mw = MainWindow::getActiveMainWindow();

        if (mw == nullptr) {
            return false;
        }

        inFile = QFileDialog::getOpenFileName(mw, QObject::tr("Import correspondences file"));

        if (inFile.isEmpty()) {
            return false;
        }
    }

    QFile in(inFile);

    if (!in.open(QFile::ReadOnly)) {
        return false;
    }

    QTextStream stream(&in);

    while (!stream.atEnd()) {
        QString line = stream.readLine();

        //replace object names with object ids
        QStringList splitted = line.split(",");

        if (splitted.size() != 2) {
            continue;
        }

        QStringList proccesed;

        for (int i = 0; i < 2; i++) {
            QStringList subsplitted = splitted[i].split(QRegExp("\\s+"), Qt::SkipEmptyParts);
            if (subsplitted.empty()) {
                proccesed << splitted[i];
                continue;
            }
            Correspondences::Types type = Correspondences::correspTypeFromString(subsplitted[0]);

            if (!Correspondences::correspTypeHasBlockId(type) or subsplitted.size() < 2) {
                proccesed << splitted[i];
                continue;
            }

            QString reconstructed = subsplitted[0];

            QString name = subsplitted[1];
            bool ok;
            qint64 id = name.toInt(&ok);

            if (!ok) {

                DataBlock* block = project->getByName(name);

                if (block == nullptr) {
                    proccesed << splitted[i];
                    continue;
                }

                reconstructed += QString(" %1").arg(block->internalId());

            } else {
                reconstructed += QString(" %1").arg(id);
            }

            for (int i = 2; i < subsplitted.size(); i++) {
                reconstructed += QString(" %1").arg(subsplitted[i]);
            }

            proccesed << reconstructed;

        }

        if (proccesed.size() != 2) {
            continue;
        }

        line = proccesed[0] + "," + proccesed[1];

        auto opt = Correspondences::GenericPair::fromString(line);

        if (!opt.has_value()) {
            continue;
        }

        if (!opt->isValid()) {
            continue;
        }

        correspSet->addCorrespondence(opt.value());
    }

    return true;
}


bool exportCorrespondancesToTxt(Project* project, qint64 correspSetId, QString const& exportFilePath) {

    if (project == nullptr) {
        return false;
    }

    CorrespondencesSet* correspSet = project->getDataBlock<CorrespondencesSet>(correspSetId);

    if (correspSet == nullptr) {
        return false;
    }

    QString outFile = exportFilePath;

    if (outFile.isEmpty()) {

        MainWindow* mw = MainWindow::getActiveMainWindow();

        if (mw == nullptr) {
            return false;
        }

        outFile = QFileDialog::getSaveFileName(mw, QObject::tr("Export correspondences file"));

        if (outFile.isEmpty()) {
            return false;
        }
    }

    QFile out(outFile);

    if (!out.open(QFile::WriteOnly)) {
        return false;
    }

    QTextStream outstream(&out);

    int nCorresps = correspSet->nCorrespondence();

    for (int i = 0; i < nCorresps; i++) {

        auto corresp = correspSet->getCorrespondence(i);

        outstream << corresp.toString() << "\n";
    }

    outstream.flush();
    out.close();

    return true;

}

} // namespace StereoVisionApp
