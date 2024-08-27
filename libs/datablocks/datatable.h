#ifndef STEREOVISIONAPP_DATATABLE_H
#define STEREOVISIONAPP_DATATABLE_H

#include "./project.h"

#include <QAbstractTableModel>

namespace StereoVisionApp {

/*!
 * \brief The DataTable class contain tabular data that can be imported from csv and used by different algorithms.
 *
 * data is assumed to be mostly static
 */
class DataTable : public DataBlock
{
    Q_OBJECT

public:
    explicit DataTable(Project* parent);
    explicit DataTable(DataBlock *parent = nullptr);

    bool hasOptimizedParameters() const override;

    QVector<QString> columns() const;
    int nRows() const;

    QVector<QVariant> getColumnData(QString colName) const;
    QVariant getData(QString colName, int row) const;

    void setData(QMap<QString, QVector<QVariant>> const& data);

Q_SIGNALS:

    void dataChanged();

protected:

    QJsonObject encodeJson() const override;
    void configureFromJson(QJsonObject const& json_data) override;

    QMap<QString, QVector<QVariant>> _data;
};

class DataTableModel : public QAbstractTableModel
{
    Q_OBJECT
public:
    DataTableModel (DataTable* data_table, QObject* parent = nullptr);

    virtual int rowCount(const QModelIndex &parent = QModelIndex()) const override;
    virtual int columnCount(const QModelIndex &parent = QModelIndex()) const override;

    virtual QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;

    virtual QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const override;

protected:

    DataTable* _data_table;
};

class DataTableFactory : public DataBlockFactory
{
    Q_OBJECT
public:
    explicit DataTableFactory(QObject* parent = nullptr);

    virtual QString TypeDescrName() const;
    virtual FactorizableFlags factorizable() const;
    virtual DataBlock* factorizeDataBlock(Project *parent = nullptr) const;

    virtual QString itemClassName() const;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_DATATABLE_H
