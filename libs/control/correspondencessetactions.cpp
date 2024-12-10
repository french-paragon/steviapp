#include "correspondencessetactions.h"

#include <QFileDialog>
#include <QFile>
#include <QTextStream>
#include <QMessageBox>

#include "control/mainwindow.h"

#include "datablocks/correspondencesset.h"
#include "datablocks/genericcorrespondences.h"

namespace StereoVisionApp {

bool importCorrespondencesFromTxt(Project* project, qint64 correspSetId, QString const& importFilePath) {

    MainWindow* mw = MainWindow::getActiveMainWindow();

    if (project == nullptr) {
        return false;
    }

    CorrespondencesSet* correspSet = project->getDataBlock<CorrespondencesSet>(correspSetId);

    if (correspSet == nullptr and correspSetId < 0) {
        qint64 id = project->createDataBlock(CorrespondencesSet::staticMetaObject.className());

        if (id < 0) {
            if (mw != nullptr) {
                QMessageBox::warning(mw,
                                     QObject::tr("Could not import correspondences"),
                                     QObject::tr("Could not create datablock in project!"));
            }
            return false;
        }

        correspSet = project->getDataBlock<CorrespondencesSet>(id);

        if (correspSet == nullptr) {
            if (mw != nullptr) {
                QMessageBox::warning(mw,
                                     QObject::tr("Could not import correspondences"),
                                     QObject::tr("Could not load created datablock from project!"));
            }
            return false;
        }

        correspSet->setObjectName("Imported correspondence set");

    } else if (correspSet == nullptr) {
        if (mw != nullptr) {
            QMessageBox::warning(mw,
                                 QObject::tr("Could not import correspondences"),
                                 QObject::tr("Could not load datablock from project!"));
        }
        return false;
    }

    QString inFile = importFilePath;

    if (inFile.isEmpty()) {

        if (mw == nullptr) {
            return false;
        }

        inFile = QFileDialog::getOpenFileName(mw, QObject::tr("Import correspondences file"));

        if (inFile.isEmpty()) { //cancelled by the user
            return false;
        }
    }

    QFile in(inFile);

    if (!in.open(QFile::ReadOnly)) {
        if (mw != nullptr) {
            QMessageBox::warning(mw,
                                 QObject::tr("Could not import correspondences"),
                                 QObject::tr("Could not open file %1!").arg(inFile));
        }
        return false;
    }

    QTextStream stream(&in);

    int nImported = 0;
    int nLines = 0;

    while (!stream.atEnd()) {
        QString line = stream.readLine();
        nLines++;

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

        nImported++;
        correspSet->addCorrespondence(opt.value());
    }

    if (nImported <= 0) {

        if (mw != nullptr) {
            QMessageBox::warning(mw,
                                 QObject::tr("Import failed"),
                                 QObject::tr("No correspondences have been imported, check the file format!"));
        }

        return false;
    }

    if (mw != nullptr and nImported < nLines) {
        QMessageBox::warning(mw,
                             QObject::tr("Not all lines could be imported"),
                             QObject::tr("Imported %1 lines of a total or %2 read line!").arg(nImported).arg(nLines));
    }

    return true;
}


bool exportCorrespondencesToTxt(Project* project, qint64 correspSetId, QString const& exportFilePath) {

    MainWindow* mw = MainWindow::getActiveMainWindow();

    if (project == nullptr) {
        return false;
    }

    CorrespondencesSet* correspSet = project->getDataBlock<CorrespondencesSet>(correspSetId);

    if (correspSet == nullptr) {
        if (mw != nullptr) {
            QMessageBox::warning(mw,
                                 QObject::tr("Could not export correspondences"),
                                 QObject::tr("Could not load datablock from project!"));
        }
        return false;
    }

    QString outFile = exportFilePath;

    if (outFile.isEmpty()) {

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
        if (mw != nullptr) {
            QMessageBox::warning(mw,
                                 QObject::tr("Could not export correspondences"),
                                 QObject::tr("Could not open file %1!").arg(outFile));
        }
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
