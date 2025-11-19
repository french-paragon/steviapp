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
#include "../utils/statusoptionalreturn.h"

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

    struct RobustificationLevel {
        enum Level {
            None = 0,
            Huber = 1,
            Cauchy = 2,
            Arctan = 3,
            Maximum = Arctan //both maximal level, and maximal level of robustification
        };
    };

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

    int robustificationLevel() const;
    QString robustificationMethod() const;
    void setRobustificationMethod(QString const& methodName);

    bool isVerbose() const;
    void setVerbose(bool verbose);

    static StatusOptionalReturn<CorrespondencesSet*> writeUVCorrespondancesToSet(
        Correspondences::UVMatchBuilder* builder1,
        std::vector<std::array<float, 2>> const& uvs1,
        Correspondences::UVMatchBuilder* builder2,
        std::vector<std::array<float, 2>> const& uvs2,
        Project* project,
        bool doNotDuplicatedNamedSet = false,
        CorrespondencesSet* correspondenceSet = nullptr
        );

    inline static StatusOptionalReturn<CorrespondencesSet*> writeUVCorrespondancesToSet(
        Correspondences::UVMatchBuilder* builder1,
        std::vector<std::array<float, 2>> const& uvs1,
        Correspondences::UVMatchBuilder* builder2,
        std::vector<std::array<float, 2>> const& uvs2,
        Project* project,
        CorrespondencesSet* correspondenceSet
        ) {
        return writeUVCorrespondancesToSet(builder1, uvs1, builder2, uvs2,project,false,correspondenceSet);
    }

Q_SIGNALS:

    void robustificationLevelChanged();
    void verboseLevelChanged();

protected:

    virtual void referedCleared(QVector<qint64> const& referedId) override;

    QJsonObject encodeJson() const override;
    void configureFromJson(QJsonObject const& data) override;

    void extendDataModel();

    QVector<Correspondences::GenericPair> _correspondences;
    int _robustificationLevel;

    bool _verbose;

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
