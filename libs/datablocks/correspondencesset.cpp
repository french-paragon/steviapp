#include "correspondencesset.h"

#include "./image.h"
#include "./localcoordinatesystem.h"

#include "./itemdatamodel.h"

namespace StereoVisionApp {

CorrespondencesSet::CorrespondencesSet(Project *parent) :
    DataBlock(parent),
    _robustificationLevel(RobustificationLevel::None),
    _verbose(false)
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

int CorrespondencesSet::robustificationLevel() const {
    return _robustificationLevel;
}
QString CorrespondencesSet::robustificationMethod() const {
    switch(_robustificationLevel) {
    case RobustificationLevel::Huber:
        return "Huber";
    case RobustificationLevel::Cauchy:
        return "Cauchy";
    case RobustificationLevel::Arctan:
        return "Arctan";
    }

    return "None";
}
void CorrespondencesSet::setRobustificationMethod(QString const& methodName) {
    QString str = methodName.toLower().trimmed();

    int level = RobustificationLevel::None;

    if (str == "huber") {
        level = RobustificationLevel::Huber;
    } else if (str == "cauchy") {
        level = RobustificationLevel::Cauchy;
    } else if (str == "arctan") {
        level = RobustificationLevel::Arctan;
    } else {
        level = RobustificationLevel::None;
    }

    if (level != _robustificationLevel) {
        _robustificationLevel = level;
        Q_EMIT robustificationLevelChanged();
        isChanged();
    }
}

bool CorrespondencesSet::isVerbose() const {
    return _verbose;
}
void CorrespondencesSet::setVerbose(bool verbose) {

    if (_verbose == verbose) {
        return;
    }

    _verbose = verbose;
    Q_EMIT verboseLevelChanged();
    isChanged();
}

StatusOptionalReturn<CorrespondencesSet*> CorrespondencesSet::writeUVCorrespondancesToSet(
    Correspondences::UVMatchBuilder* builder1,
    std::vector<std::array<float, 2>> const& uvs1,
    Correspondences::UVMatchBuilder* builder2,
    std::vector<std::array<float, 2>> const& uvs2,
    Project* project,
    bool doNotDuplicatedNamedSet,
    CorrespondencesSet* correspondenceSet
    ) {

    if (builder1 == nullptr or builder2 == nullptr) {
        return StatusOptionalReturn<CorrespondencesSet*>::error("Null builder!");
    }

    if (project == nullptr) {
        return StatusOptionalReturn<CorrespondencesSet*>::error("Null project!");
    }

    CorrespondencesSet* correspSet = correspondenceSet;

    if (correspSet == nullptr) {

        QString expectedName = QString("%1 - %2 correspondence set").arg(builder1->targetTitle(), builder2->targetTitle());

        if (!doNotDuplicatedNamedSet) {
            correspSet = project->getDataBlockByName<CorrespondencesSet>(expectedName);
        }

        if (correspSet == nullptr) {
            qint64 id = project->createDataBlock(CorrespondencesSet::staticMetaObject.className());

            if (id < 0) {
                return StatusOptionalReturn<CorrespondencesSet*>::error("Could not create correspondences set in project!");
            }

            correspSet = project->getDataBlock<CorrespondencesSet>(id);
        }

        if (correspSet == nullptr) {
            return StatusOptionalReturn<CorrespondencesSet*>::error("Could not load created correspondences set in project!");
        }

        correspSet->setObjectName(expectedName);
    }


    for (int i = 0; i < uvs1.size(); i++) {

        auto& [u1, v1] = uvs1[i];
        auto& [u2, v2] = uvs2[i];

        correspSet->addCorrespondence(Correspondences::GenericPair{builder1->correspondanceFromUV(u1,v1),
                                                                   builder2->correspondanceFromUV(u2,v2)});
    }

    return correspSet;
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

    switch (_robustificationLevel) {
    case RobustificationLevel::Huber:
        obj.insert("robustification", "huber");
        break;
    case RobustificationLevel::Cauchy:
        obj.insert("robustification", "cauchy");
        break;
    case RobustificationLevel::Arctan:
        obj.insert("robustification", "arctan");
        break;
    }

    obj.insert("verbose", _verbose);

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

    _robustificationLevel = RobustificationLevel::None;

    if (data.contains("robustification")) {
        QJsonValue val = data.value("robustification");

        QString str = val.toString().toLower().trimmed();

        if (str == "huber") {
            _robustificationLevel = RobustificationLevel::Huber;
        } else if (str == "cauchy") {
            _robustificationLevel = RobustificationLevel::Cauchy;
        } else if (str == "arctan") {
            _robustificationLevel = RobustificationLevel::Arctan;
        } else {
            int level = val.toInt(RobustificationLevel::None);

            if (level >= RobustificationLevel::None and level <= RobustificationLevel::Maximum) {
                _robustificationLevel = level;
            }
        }
    }

    if (data.contains("verbose")) {
        _verbose = data.value("verbose").toBool();
    } else {
        _verbose = false;
    }
}

void CorrespondencesSet::extendDataModel() {


    ItemDataModel::Category* optCat = _dataModel->addCategory(tr("Optimizer properties"));

    optCat->addCatProperty<bool, DataBlock, false, ItemDataModel::ItemPropertyDescription::PassByValueSignal>(tr("Enabled"),
                                                                                                              &DataBlock::isEnabled,
                                                                                                              &DataBlock::setEnabled,
                                                                                                              &DataBlock::isEnabledChanged);

    auto* robustificationProp = optCat->addCatProperty<QString, CorrespondencesSet, true, ItemDataModel::ItemPropertyDescription::NoValueSignal>(
        tr("Robustificaion"),
        &CorrespondencesSet::robustificationMethod,
        &CorrespondencesSet::setRobustificationMethod,
        &CorrespondencesSet::robustificationLevelChanged);

    robustificationProp->setOptions(QStringList{"None", "Huber", "Cauchy", "Arctan"});

    optCat->addCatProperty<bool, CorrespondencesSet, false, ItemDataModel::ItemPropertyDescription::NoValueSignal>(
        tr("Verbose"),
        &CorrespondencesSet::isVerbose,
        &CorrespondencesSet::setVerbose,
        &CorrespondencesSet::verboseLevelChanged);

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
