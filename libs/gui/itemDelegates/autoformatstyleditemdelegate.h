#ifndef AUTOFORMATSTYLEDITEMDELEGATE_H
#define AUTOFORMATSTYLEDITEMDELEGATE_H

#include <QStyledItemDelegate>

namespace StereoVisionApp {

/*!
 * \brief The AutoFormatStyledItemDelegate class act like QStyledItemDelegate, but with additional support for some auto format types used in StereoVisionApp
 */
class AutoFormatStyledItemDelegate : public QStyledItemDelegate
{
    Q_OBJECT
public:
    explicit AutoFormatStyledItemDelegate(QObject *parent = nullptr);

    virtual QWidget* createEditor(QWidget *parent, const QStyleOptionViewItem &option, const QModelIndex &index) const override;
    virtual void setEditorData(QWidget *editor, const QModelIndex &index) const override;
    virtual void setModelData(QWidget *editor, QAbstractItemModel *model, const QModelIndex &index) const override;

Q_SIGNALS :
};

} // namespace StereoVisionApp

#endif // AUTOFORMATSTYLEDITEMDELEGATE_H
