#ifndef STEREOVISIONAPP_FIXEDSTEREOPLUSCOLORSEQUENCE_H
#define STEREOVISIONAPP_FIXEDSTEREOPLUSCOLORSEQUENCE_H


#include "./project.h"

#include <QAbstractTableModel>

namespace StereoVisionApp {

class FixedStereoPlusColorSequence : public DataBlock
{
	Q_OBJECT
public:
	FixedStereoPlusColorSequence(Project* parent);

	struct ImageTriplet {
		QString StereoLeftImgPath;
		QString StereoRightImgPath;
		QString ColorImagePath;
	};

	QString baseFolder() const;
	QVector<ImageTriplet> imgsTriplets() const;

	void setImgsLists(const QString &baseFolder, QVector<ImageTriplet> const& images);

	QAbstractItemModel* getImageList() const;

Q_SIGNALS:

	void imgListChanged();

protected:

	QJsonObject encodeJson() const override;
	void configureFromJson(QJsonObject const& data) override;

	QString _baseFolder;
	QVector<ImageTriplet> _imgsTriplets;

	QAbstractItemModel* _imgList;
};

class FixedStereoPlusColorSequenceImageList : public QAbstractTableModel
{
	Q_OBJECT
public:
	explicit FixedStereoPlusColorSequenceImageList(FixedStereoPlusColorSequence* parent);

	int rowCount(const QModelIndex &parent = QModelIndex()) const override;
	int columnCount(const QModelIndex &parent = QModelIndex()) const override;
	QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;

	QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const override;


protected:

	FixedStereoPlusColorSequence* _sequence;

	friend class FixedStereoPlusColorSequence;

};

class FixedStereoPlusColorSequenceFactory : public DataBlockFactory
{
	Q_OBJECT
public:
	explicit FixedStereoPlusColorSequenceFactory(QObject* parent = nullptr);

	QString TypeDescrName() const override;
	FactorizableFlags factorizable() const override;
	DataBlock* factorizeDataBlock(Project *parent = nullptr) const override;

	QString itemClassName() const override;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_FIXEDSTEREOPLUSCOLORSEQUENCE_H
