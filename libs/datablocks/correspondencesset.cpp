#include "correspondencesset.h"

#include "./image.h"
#include "./localcoordinatesystem.h"

#include "./itemdatamodel.h"

namespace StereoVisionApp {

CorrespondencesSet::CorrespondencesSet(Project *parent) :
    DataBlock(parent)
{
    extendDataModel();
}

void CorrespondencesSet::addCorrespondence(Correspondences::GenericPair const& correspondence) {

    if (!correspondence.isValid()) {
        return;
    }

    auto idxs = correspondence.datablocksIds();

    Project* proj = getProject();

    if (proj == nullptr) {
        _correspondences.push_back(correspondence);
        return;
    }

    DataBlock* first = proj->getById(idxs.first);

    if (first != nullptr) {
        addRefered({idxs.first});
    }

    DataBlock* second = proj->getById(idxs.second);

    if (second != nullptr and second != first) {
        addRefered({idxs.second});
    }

    _correspondences.push_back(correspondence);
}
void CorrespondencesSet::removeCorrespondence(int i) {

    if (i < 0 or i >= _correspondences.size()) {
        return;
    }

    Correspondences::GenericPair& pair = _correspondences[i];

    auto idxs = pair.datablocksIds();

    _correspondences.remove(i);

    if (idxs.first >= 0) {
        auto list = getCorrespondencesOfBlock(idxs.first);

        if (list.empty()) {
            removeRefered({idxs.first});
        }
    }

    if (idxs.second >= 0 and idxs.second != idxs.first) {
        auto list = getCorrespondencesOfBlock(idxs.second);

        if (list.empty()) {
            removeRefered({idxs.second});
        }
    }

}


void CorrespondencesSet::clearOptimized() {
    return; //Nothing to optimize
}
bool CorrespondencesSet::hasOptimizedParameters() const {
    return false;
}

QJsonObject CorrespondencesSet::getJsonRepresentation() const {
    return encodeJson();
}
void CorrespondencesSet::setParametersFromJsonRepresentation(QJsonObject const& rep) {
    configureFromJson(rep);
}

void CorrespondencesSet::referedCleared(QVector<qint64> const& referedId) {

    if (referedId.size() != 1) {
        return;
    }

    qint64 blockId = referedId[0];

    for (int i = _correspondences.size()-1; i >= 0; i++) {
        if (_correspondences[i].hasDatablockId(blockId)) {
            _correspondences.remove(i);
        }
    }
}

QJsonObject CorrespondencesSet::encodeJson() const {

    QJsonObject obj;

    QJsonArray correspondences;

    for (Correspondences::GenericPair const& pair : _correspondences) {
        correspondences.push_back(pair.toString());
    }

    obj.insert("correspondences", correspondences);

    return obj;
}

void CorrespondencesSet::configureFromJson(QJsonObject const& data) {

    _correspondences.clear();

    if (data.contains("correspondences")) {
        QJsonArray arr = data.value("correspondences").toArray();
        _correspondences.reserve(arr.size());

        for (QJsonValue const& v : arr) {
            QString str = v.toString();

            std::optional<Correspondences::GenericPair> pair = Correspondences::GenericPair::fromString(str);

            if (!pair.has_value()) {
                continue;
            }

            if (!pair->isValid()) {
                continue;
            }

            _correspondences.push_back(pair.value());
        }
    }
}

void CorrespondencesSet::extendDataModel() {

}


CorrespondencesSetFactory::CorrespondencesSetFactory(QObject* parent) :
    DataBlockFactory(parent)
{

}

QString CorrespondencesSetFactory::TypeDescrName() const {
    return tr("Correspondences set");
}
DataBlockFactory::FactorizableFlags CorrespondencesSetFactory::factorizable() const {
    return DataBlockFactory::RootDataBlock;
}
DataBlock* CorrespondencesSetFactory::factorizeDataBlock(Project *parent) const {
    return new CorrespondencesSet(parent);
}

QString CorrespondencesSetFactory::itemClassName() const {
    return CorrespondencesSet::staticMetaObject.className();
}


} // namespace StereoVisionApp
