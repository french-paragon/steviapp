#include "datatableactions.h"

#include "datablocks/project.h"
#include "datablocks/datatable.h"

#include "control/mainwindow.h"

#include "gui/datatablevieweditor.h"

#include <QDebug>

#include <QFileDialog>
#include <QMessageBox>
#include <QStandardPaths>

#include <QFile>
#include <QFileInfo>

namespace StereoVisionApp {

bool openDataTableFromCsv(Project* p) {

    if (p == nullptr) {
        return false;
    }

    QString functionName = "openDataTableFromCsv";

    MainWindow* mw = MainWindow::getActiveMainWindow();

    if (mw == nullptr) {
        qDebug() << functionName << "missing main windows";
        return false;
    }

    QString inFilePath = QFileDialog::getOpenFileName(mw,
                                 QObject::tr("Open csv data"),
                                 QStandardPaths::standardLocations(QStandardPaths::DocumentsLocation).first(),
                                 QObject::tr("csv files (*.csv *.txt *.dat);;all files (*.*)"));


    if (inFilePath.isEmpty()) {
        return false;
    }

    qint64 id = p->createDataBlock(DataTable::staticMetaObject.className());
    DataTable* dataTable = p->getDataBlock<DataTable>(id);

    if (dataTable == nullptr) {
        QMessageBox::warning(mw, QObject::tr("Could not create datablock"), QObject::tr("Internal project error"));
        return false;
    }

    QFile file(inFilePath);
    QFileInfo fileInfo(inFilePath);

    if (!file.exists()) {
        QMessageBox::warning(mw, QObject::tr("Could not import data"), QObject::tr("File %1 does not exist").arg(inFilePath));
        return false;
    }

    if (!file.open(QFile::ReadOnly)) {
        QMessageBox::warning(mw, QObject::tr("Could not import data"), QObject::tr("File %1 could not be opened").arg(inFilePath));
        return false;
    }

    QString data = QString::fromLocal8Bit(file.readAll());

    file.close();

    QStringList splitted = data.split("\n");

    QStringList cols = splitted.first().split(",", Qt::KeepEmptyParts);

    int nLines = splitted.size()-1;

    QMap<QString, QVector<QVariant>> data_struct;

    for (QString & key : cols) {
        key = key.trimmed();
        data_struct.insert(key, QVector<QVariant>(nLines));
    }

    for (int i = 0; i < nLines; i++) {
        QStringList values = splitted[i+1].split(",", Qt::KeepEmptyParts);

        for (int j = 0; j < std::min(values.size(), cols.size()); j++) {
            QString& colName = cols[j];
            data_struct[colName][i] = QVariant(values[j]);
        }
    }

    dataTable->setObjectName(fileInfo.fileName());
    dataTable->setData(data_struct);

    return true;
}

bool openDataTable(DataBlock* d) {

    DataTable* dataTable = qobject_cast<DataTable*>(d);

    if (dataTable == nullptr) {
        return false;
    }

    MainWindow* mw = MainWindow::getActiveMainWindow();

    if (mw == nullptr) {
        return false;
    }

    Editor* e = mw->openEditor(DataTableViewEditor::staticMetaObject.className());

    DataTableViewEditor* dtve = qobject_cast<DataTableViewEditor*>(e);

    if (dtve == nullptr) {
        return false;
    }

    dtve->setDataTable(dataTable);

    return true;
}

bool removeDataTable(DataBlock* d) {

    DataTable* dataTable = qobject_cast<DataTable*>(d);

    if (dataTable == nullptr) {
        return false;
    }

    Project* p = d->getProject();

    if (p == nullptr) {
        return false;
    }

    p->clearById(dataTable->internalId());

    return true;
}

bool exportDataTableToCsv(DataBlock* d) {

    QString functionName = "exportDataTableToCsv";

    DataTable* dataTable = qobject_cast<DataTable*>(d);

    if (dataTable == nullptr) {
        return false;
    }

    MainWindow* mw = MainWindow::getActiveMainWindow();

    if (mw == nullptr) {
        qDebug() << functionName << "missing main windows";
        return false;
    }

    QString saveFilePath = QFileDialog::getSaveFileName(mw,
                                 QObject::tr("Save to csv data"),
                                 QStandardPaths::standardLocations(QStandardPaths::DocumentsLocation).first(),
                                 QObject::tr("csv files (*.csv *.txt *.dat);;all files (*.*)"));


    if (saveFilePath.isEmpty()) {
        qDebug() << functionName << "empty save file path";
        return false;
    }

    if (!saveFilePath.endsWith(".csv", Qt::CaseInsensitive) and
            !saveFilePath.endsWith(".txt", Qt::CaseInsensitive) and
            !saveFilePath.endsWith(".dat", Qt::CaseInsensitive)) {

        saveFilePath += ".csv";
    }

    QFile file(saveFilePath);
    QFileInfo fileInfo(saveFilePath);

    if (!file.open(QFile::WriteOnly)) {
        QMessageBox::warning(mw, QObject::tr("Could not export data"), QObject::tr("File %1 could not be opened").arg(saveFilePath));
        return false;
    }

    QTextStream stream(&file);

    QVector<QString> columns = dataTable->columns();
    QVector<QVector<QVariant>> colsDatas;
    colsDatas.reserve(columns.size());

    for (QString const& col : columns) {
        colsDatas.push_back(dataTable->getColumnData(col));
    }

    int nRows = dataTable->nRows();

    for (int i = 0; i < columns.size(); i++) {
        if (i > 0) {
            stream << ',';
        }
        stream << columns[i];
    }
    stream << Qt::endl;

    for (int i = 0; i < nRows; i++) {

        for (int c = 0; c < columns.size(); c++) {
            if (c > 0) {
                stream << ',';
            }

            if (colsDatas[c].size() > i) {
                stream << colsDatas[c][i].toString();
            }
        }
        stream << Qt::endl;

    }

    file.close();

    qDebug() << functionName << "data written to: " << saveFilePath;

    return true;

}

} // namespace StereoVisionApp
