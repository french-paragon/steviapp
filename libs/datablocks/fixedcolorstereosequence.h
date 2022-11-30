#ifndef STEREOVISIONAPP_FIXEDCOLORSTEREOSEQUENCE_H
#define STEREOVISIONAPP_FIXEDCOLORSTEREOSEQUENCE_H

#include "./project.h"

namespace StereoVisionApp {

class FixedColorStereoSequence : public DataBlock
{
	Q_OBJECT
public:
	FixedColorStereoSequence(Project* parent);

	struct ImagePair {
		QString StereoLeftImgPath;
		QString StereoRightImgPath;
	};


	QString baseFolder() const;
	QVector<ImagePair> imgsPairs() const;

	void setImgsLists(const QString &baseFolder, QVector<ImagePair> const& images);

	QAbstractItemModel* getImageList() const;
Q_SIGNALS:

	void imgListChanged();

protected:

	QJsonObject encodeJson() const override;
	void configureFromJson(QJsonObject const& data) override;

	QString _baseFolder;
	QVector<ImagePair> _imgsPairs;

	QAbstractItemModel* _imgList;
};

class FixedColorStereoSequenceImageList : public QAbstractTableModel
{
	Q_OBJECT
public:
	explicit FixedColorStereoSequenceImageList(FixedColorStereoSequence* parent);

	int rowCount(const QModelIndex &parent = QModelIndex()) const override;
	int columnCount(const QModelIndex &parent = QModelIndex()) const override;
	QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;

	QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const override;


protected:

	FixedColorStereoSequence* _sequence;

	friend class FixedStereoPlusColorSequence;

};

class FixedColorStereoSequenceFactory : public DataBlockFactory
{
	Q_OBJECT
public:
	explicit FixedColorStereoSequenceFactory(QObject* parent = nullptr);

	QString TypeDescrName() const override;
	FactorizableFlags factorizable() const override;
	DataBlock* factorizeDataBlock(Project *parent = nullptr) const override;

	QString itemClassName() const override;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_FIXEDCOLORSTEREOSEQUENCE_H
