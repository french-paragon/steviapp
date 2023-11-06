#include "datatableactionmanager.h"

#include "../datablocks/datatable.h"

#include "./datatableactions.h"

#include <QAction>

namespace StereoVisionApp {

DataTableActionManager::DataTableActionManager(QObject* parent) :
    DatablockActionManager(parent)
{

}

QString DataTableActionManager::ActionManagerClassName() const {
    return DataTableActionManager::staticMetaObject.className();
}

QString DataTableActionManager::itemClassName() const {
    return DataTable::staticMetaObject.className();
}

QList<QAction*> DataTableActionManager::factorizeClassContextActions(QObject* parent, Project* p) const {

    QAction* openFromCSV = new QAction("open from csv", parent);

    connect(openFromCSV, &QAction::triggered, p, [p] () {
        openDataTableFromCsv(p);
    });

    return {openFromCSV};

}

QList<QAction*> DataTableActionManager::factorizeItemContextActions(QObject* parent, DataBlock* d) const {


    QAction* viewData = new QAction("view data", parent);

    connect(viewData, &QAction::triggered, d, [d] () {
        openDataTable(d);
    });


    QAction* export2csv = new QAction("export to csv", parent);

    connect(export2csv, &QAction::triggered, d, [d] () {
        exportDataTableToCsv(d);
    });


    QAction* removeData = new QAction("delete", parent);

    connect(removeData, &QAction::triggered, d, [d] () {
        removeDataTable(d);
    });

    return {viewData, export2csv, removeData};

}

} // namespace StereoVisionApp
