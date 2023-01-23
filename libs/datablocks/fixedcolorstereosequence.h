#ifndef STEREOVISIONAPP_FIXEDCOLORSTEREOSEQUENCE_H
#define STEREOVISIONAPP_FIXEDCOLORSTEREOSEQUENCE_H

#include "./project.h"
#include "./exportablestereosequence.h"

namespace StereoVisionApp {

class FixedColorStereoSequence : public ExportableStereoSequence
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

	void setLeftViewId(qint64 id);
	void setRightViewId(qint64 id);

	qint64 leftViewId() const;
	qint64 rightViewId() const;

Q_SIGNALS:

	void imgListChanged();

	void leftViewIdChanged(qint64 id);
	void rightViewIdChanged(qint64 id);

protected:

	QJsonObject encodeJson() const override;
	void configureFromJson(QJsonObject const& data) override;

	void referedCleared(QVector<qint64> const& referedId) override;

	QString _baseFolder;
	QVector<ImagePair> _imgsPairs;

	QAbstractItemModel* _imgList;

	qint64 _leftViewId;
	qint64 _rightViewId;
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
