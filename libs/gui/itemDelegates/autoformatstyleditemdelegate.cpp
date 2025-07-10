#include "autoformatstyleditemdelegate.h"

#include "utils/editablefloatingpointblock.h"

#include "gui/inputsWidgets/floatingpointblockeditspinbox.h"

namespace StereoVisionApp {

AutoFormatStyledItemDelegate::AutoFormatStyledItemDelegate(QObject *parent)
    : QStyledItemDelegate{parent}
{}

QWidget* AutoFormatStyledItemDelegate::createEditor(QWidget *parent, const QStyleOptionViewItem &option, const QModelIndex &index) const {
    QVariant data = index.data(Qt::EditRole);

    if (data.userType() == qMetaTypeId<EditableFloatingPointBlock>()) {
        FloatingPointBlockEditSpinBox* editor = new FloatingPointBlockEditSpinBox(parent);
        return editor;
    }

    return QStyledItemDelegate::createEditor(parent, option, index);
}

void AutoFormatStyledItemDelegate::setEditorData(QWidget *editor, const QModelIndex &index) const {
    QVariant data = index.data(Qt::EditRole);

    if (data.userType() == qMetaTypeId<EditableFloatingPointBlock>()) {
        FloatingPointBlockEditSpinBox* floating_point_block_editor = qobject_cast<FloatingPointBlockEditSpinBox*>(editor);
        floating_point_block_editor->setEditableFloatingPointBlock(qvariant_cast<EditableFloatingPointBlock>(data));
        double val = floating_point_block_editor->value();
        return;
    }

    QStyledItemDelegate::setEditorData(editor, index);

}

void AutoFormatStyledItemDelegate::setModelData(QWidget *editor, QAbstractItemModel *model, const QModelIndex &index) const {

    FloatingPointBlockEditSpinBox* floating_point_block_editor = qobject_cast<FloatingPointBlockEditSpinBox*>(editor);

    if (floating_point_block_editor != nullptr) {
        model->setData(index, QVariant::fromValue(floating_point_block_editor->floatingPointBlock()));
        return;
    }

    QStyledItemDelegate::setModelData(editor, model, index);
}

} // namespace StereoVisionApp
