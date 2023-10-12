#ifndef STEREOVISIONAPP_DATATABLEVIEWEDITOR_H
#define STEREOVISIONAPP_DATATABLEVIEWEDITOR_H

#include "./editor.h"

class QTreeView;
class QAbstractItemModel;

namespace StereoVisionApp {

class DataBlock;

class DataTableViewEditor : public Editor
{
    Q_OBJECT
public:
    DataTableViewEditor(QWidget *parent = nullptr);

    void setDataTable(DataBlock* block);

protected:

    QAbstractItemModel* _model;
    QTreeView* _view;
};

class DataTableViewEditorFactory : public EditorFactory
{
    Q_OBJECT
public:
    explicit DataTableViewEditorFactory(QObject *parent = nullptr);


    virtual QString TypeDescrName() const override;
    virtual QString itemClassName() const override;
    virtual Editor* factorizeEditor(QWidget* parent) const override;

};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_DATATABLEVIEWEDITOR_H
