#ifndef STEREOVISIONAPP_CORRESPONDANCESSET_H
#define STEREOVISIONAPP_CORRESPONDANCESSET_H

#include "./project.h"
#include "./floatparameter.h"

#include <QPointF>
#include <QVector3D>
#include <QSet>
#include <QString>

#include <eigen3/Eigen/Core>

#include <optional>

#include "./genericcorrespondences.h"

namespace StereoVisionApp {

class Image;
class LocalCoordinateSystem;

/*!
 * \brief The CorrespondancesSet class represent arbitrary correspondances between datablocks in the project
 */
class CorrespondencesSet : public DataBlock
{
    Q_OBJECT
public:
    CorrespondencesSet(Project* parent = nullptr);

    void addCorrespondence(Correspondences::GenericPair const& correspondence);
    int nCorrespondence() const {
        return _correspondences.size();
    }
    inline Correspondences::GenericPair & getCorrespondence(int i) {
        return _correspondences[i];
    }
    inline Correspondences::GenericPair const& getCorrespondence(int i) const {
        return _correspondences[i];
    }
    void removeCorrespondence(int i);

    inline QVector<Correspondences::GenericPair> getCorrespondencesOfBlock(qint64 id) {
        QVector<Correspondences::GenericPair> ret;

        for (Correspondences::GenericPair const& pair : _correspondences) {
            if (pair.hasDatablockId(id)) {
                ret.push_back(pair);
            }
        }

        return ret;
    }

    template<Correspondences::Types C1, Correspondences::Types C2>
    QVector<Correspondences::TypedPair<C1,C2>> getCorrespondancesOfType() const {
        QVector<Correspondences::TypedPair<C1,C2>> ret;
        ret.reserve(_correspondences.size()/2 + 1);

        for (Correspondences::GenericPair const& pair : _correspondences) {
            if (pair.holdsCorrespondancesType<C1,C2>()) {
                ret.push_back(pair.getTypedPair<C1,C2>().value());
            }
        }

        return ret;
    }

    void clearOptimized() override;
    bool hasOptimizedParameters() const override;

    QJsonObject getJsonRepresentation() const override;
    void setParametersFromJsonRepresentation(QJsonObject const& rep) override;

Q_SIGNALS:

protected:

    virtual void referedCleared(QVector<qint64> const& referedId) override;

    QJsonObject encodeJson() const override;
    void configureFromJson(QJsonObject const& data) override;

    void extendDataModel();

    QVector<Correspondences::GenericPair> _correspondences;
};


class CorrespondencesSetFactory : public DataBlockFactory
{
    Q_OBJECT
public:
    explicit CorrespondencesSetFactory(QObject* parent = nullptr);

    virtual QString TypeDescrName() const;
    virtual FactorizableFlags factorizable() const;
    virtual DataBlock* factorizeDataBlock(Project *parent = nullptr) const;

    virtual QString itemClassName() const;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_CORRESPONDANCESSET_H
