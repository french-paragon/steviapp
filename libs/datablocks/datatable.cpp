#include "datatable.h"

#include <QJsonObject>
#include <QJsonValue>
#include <QJsonArray>

namespace StereoVisionApp {

DataTable::DataTable(Project *parent) :
    DataBlock(parent)
{

}

bool DataTable::hasOptimizedParameters() const {
    return false;
}

QVector<QString> DataTable::columns() const {
    return _data.keys().toVector();
}

int DataTable::nRows() const {

    int maxDataSize = 0;

    for (QVector<QVariant> const& col : _data) {
        if (col.size() > maxDataSize) {
            maxDataSize = col.size();
        }
    }

    return maxDataSize;

}

QVector<QVariant> DataTable::getColumnData(QString colName) const {
    return _data.value(colName, {});
}

QVariant DataTable::getData(QString colName, int row) const {

    if (!_data.contains(colName)) {
        return QVariant();
    }

    QVector<QVariant> const& col = _data[colName];

    if (row >= col.size() or row < 0) {
        return QVariant();
    }

    return col[row];
}

void DataTable::setData(QMap<QString, QVector<QVariant>> const& data) {

    _data = data;
    Q_EMIT dataChanged();

}

QJsonObject DataTable::encodeJson() const {

    QJsonObject obj;

    for (QString const& colName : _data.keys()) {
        QJsonArray values;

        for (QVariant const& var : _data[colName]) {

            values.push_back(QJsonValue(var.toString()));
        }

        obj.insert(colName, values);
    }

    return obj;

}
void DataTable::configureFromJson(QJsonObject const& json_data) {

    QMap<QString, QVector<QVariant>> data;

    for (QString key : json_data.keys()) {

        QJsonValue val = json_data.value(key);

        if (!val.isArray()) {
            continue;
        }

        QJsonArray arr = val.toArray();
        QVector<QVariant> dat;
        dat.reserve(arr.size());

        for (QJsonValue const& val : arr) {
            dat.push_back(QVariant(val.toString()));
        }

        data.insert(key, dat);
    }

    setData(data);

}

DataTableModel::DataTableModel (DataTable* data_table, QObject* parent) :
    QAbstractTableModel(parent),
    _data_table(data_table)
{

    if (_data_table != nullptr) {
        connect(_data_table, &QObject::destroyed, this, [this] () {
            beginResetModel();
            _data_table = nullptr;
            endResetModel();
        });
    }

}

int DataTableModel::rowCount(const QModelIndex &parent) const {

    if (_data_table == nullptr) {
        return 0;
    }

    return _data_table->nRows();

}
int DataTableModel::columnCount(const QModelIndex &parent) const {

    if (_data_table == nullptr) {
        return 0;
    }

    return _data_table->columns().size();

}

QVariant DataTableModel::data(const QModelIndex &index, int role) const {

    if (index == QModelIndex() or _data_table == nullptr) {
        return QVariant();
    }

    QString colName = _data_table->columns()[index.column()];

    switch (role) {

    case Qt::DisplayRole:

        return _data_table->getData(colName, index.row());

    default:
        break;
    }

    return QVariant();

}

QVariant DataTableModel::headerData(int section, Qt::Orientation orientation, int role) const {

    if (_data_table == nullptr) {
        return QVariant();
    }

    QString colName = (orientation == Qt::Horizontal) ? _data_table->columns()[section] : "";

    switch (role) {
    case Qt::DisplayRole:
        return colName;
    default:
        break;
    }

    return QVariant();

}

DataTableFactory::DataTableFactory(QObject* parent) :
    DataBlockFactory(parent)
{

}

QString DataTableFactory::TypeDescrName() const {
    return tr("Data Table");
}
DataTableFactory::FactorizableFlags DataTableFactory::factorizable() const {
    return RootDataBlock;
}
DataBlock* DataTableFactory::factorizeDataBlock(Project *parent) const {
    return new DataTable(parent);
}

QString DataTableFactory::itemClassName() const {
    return DataTable::staticMetaObject.className();
}

} // namespace StereoVisionApp
