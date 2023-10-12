#include "datatablevieweditor.h"

#include <QVBoxLayout>
#include <QTableView>
#include <QTreeView>

#include "datablocks/datatable.h"

namespace StereoVisionApp {

DataTableViewEditor::DataTableViewEditor(QWidget *parent) :
    Editor(parent)
{

    _model = nullptr;

    QVBoxLayout* layout = new QVBoxLayout();
    layout->setMargin(0);

    _view = new QTreeView(this);

    layout->addWidget(_view);

    setLayout(layout);

}

void DataTableViewEditor::setDataTable(DataBlock* block) {

    DataTable* table = qobject_cast<DataTable*>(block);

    DataTableModel* dataTableModel = new DataTableModel(table, this);

    if (_model != nullptr) {
        _model->deleteLater();
    }

    _model = dataTableModel;
    _view->setModel(_model);

}

DataTableViewEditorFactory::DataTableViewEditorFactory(QObject *parent) :
    EditorFactory(parent)
{

}

QString DataTableViewEditorFactory::TypeDescrName() const {
    return tr("Data table editor");
}
QString DataTableViewEditorFactory::itemClassName() const {
    return DataTableViewEditor::staticMetaObject.className();
}
Editor* DataTableViewEditorFactory::factorizeEditor(QWidget* parent) const {
    return new DataTableViewEditor(parent);
}

} // namespace StereoVisionApp
