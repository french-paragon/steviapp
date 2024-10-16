#include "sbagraphreductor.h"

#include "datablocks/project.h"
#include "datablocks/image.h"
#include "datablocks/landmark.h"
#include "datablocks/localcoordinatesystem.h"

#include <QVector>

namespace StereoVisionApp {

SBAGraphReductor::SBAGraphReductor(int min_img_obs,
								   int min_pt_obs,
								   bool count_self_obs_pos,
								   bool count_self_obs_rot) :
	_min_img_obs(min_img_obs),
	_min_pt_obs(min_pt_obs),
	_count_self_obs_pos(count_self_obs_pos),
	_count_self_obs_rot(count_self_obs_rot)
{

}


SBAGraphReductor::elementsSet SBAGraphReductor::reduceGraph(Project* p, bool initializedOnly) const {

	QVector<qint64> imgs_v = p->getIdsByClass(ImageFactory::imageClassName());
	QVector<qint64> lmks_v = p->getIdsByClass(LandmarkFactory::landmarkClassName());
	QVector<qint64> lcos_v = p->getIdsByClass(LocalCoordinateSystem::staticMetaObject.className());

	QSet<qint64> imgs(imgs_v.begin(), imgs_v.end());
	QSet<qint64> lmks(lmks_v.begin(), lmks_v.end());
	QSet<qint64> lcos(lcos_v.begin(), lcos_v.end());


	QVector<qint64> excludedImgs;
	QVector<qint64> excludedLmks;
	QVector<qint64> excludedLcs;

	excludedImgs.reserve(imgs.size());
	excludedLmks.reserve(lmks.size());
	excludedLcs.reserve(lcos.size());

	int eImgsPos = 0;
	int eLmksPos = 0;
	int eLCosPos = 0;

	while (true) {

		for (qint64 id : imgs) {
			Image* im = qobject_cast<Image*>(p->getById(id));
			if (im == nullptr) {
				continue;
			}

			if (initializedOnly and im->optimizationStep() < DataBlock::Initialized) {
				continue;
			}

			int connections = im->countPointsRefered(excludedLmks);

			if (_count_self_obs_pos and _count_self_obs_rot) {
				if (im->xCoord().isSet() or im->yCoord().isSet() or im->zCoord().isSet() or
					im->xRot().isSet() or im->yRot().isSet() or im->zRot().isSet()) {
                    connections += 3;
				}
			} else if (_count_self_obs_pos) {
				if (im->xCoord().isSet() or im->yCoord().isSet() or im->zCoord().isSet()) {
					connections++;
				}
			} else if (_count_self_obs_rot) {
				if (im->xRot().isSet() or im->yRot().isSet() or im->zRot().isSet()) {
					connections++;
				}
			}

			if (connections < _min_img_obs or im->isEnabled() == false) {
				excludedImgs.push_back(id);
			}
		}

		for (qint64 id : lcos) {
			LocalCoordinateSystem* lcs = p->getDataBlock<LocalCoordinateSystem>(id);
			if (lcs == nullptr) {
				continue;
			}

			if (initializedOnly and lcs->optimizationStep() < DataBlock::Initialized) {
				continue;
			}

			int connections = lcs->countPointsRefered(excludedLmks);

			if (_count_self_obs_pos and _count_self_obs_rot) {
				if (lcs->xCoord().isSet() or lcs->yCoord().isSet() or lcs->zCoord().isSet() or
					lcs->xRot().isSet() or lcs->yRot().isSet() or lcs->zRot().isSet()) {
                    connections += 3;
				}
			} else if (_count_self_obs_pos) {
				if (lcs->xCoord().isSet() or lcs->yCoord().isSet() or lcs->zCoord().isSet()) {
					connections++;
				}
			} else if (_count_self_obs_rot) {
				if (lcs->xRot().isSet() or lcs->yRot().isSet() or lcs->zRot().isSet()) {
					connections++;
				}
			}

			if (connections < _min_img_obs or lcs->isEnabled() == false) {
				excludedLcs.push_back(id);
			}
		}

		for (qint64 id : lmks) {
			Landmark* lm =  qobject_cast<Landmark*>(p->getById(id));

			if (lm == nullptr) {
				continue;
			}

			if (initializedOnly and lm->optimizationStep() < DataBlock::Initialized) {
				continue;
			}

			int connections = lm->countImagesRefering(excludedImgs);
			connections += lm->countLocalCoordinateSystemsRefering(excludedLcs);

			if (_count_self_obs_pos) {
				if (lm->xCoord().isSet() or lm->yCoord().isSet() or lm->zCoord().isSet()) {
					connections++;
				}
			}

			if (connections < _min_pt_obs or lm->isEnabled() == false) {
				excludedLmks.push_back(id);
			}
		}

		if (eImgsPos == excludedImgs.size() and eLmksPos == excludedLmks.size()) { //we reach maximal subgraph matching number of observations criteriea
			break;
		}

		for (;eImgsPos < excludedImgs.size(); eImgsPos++) {
			imgs.remove(excludedImgs[eImgsPos]);
		}

		for (;eLmksPos < excludedLmks.size(); eLmksPos++) {
			lmks.remove(excludedLmks[eLmksPos]);
		}

		for (;eLCosPos < excludedLcs.size(); eLCosPos++) {
			lcos.remove(excludedLcs[eLCosPos]);
		}

	}

	return {imgs,
			lmks,
			lcos,
			QSet<qint64>(excludedImgs.begin(), excludedImgs.end()),
			QSet<qint64>(excludedLmks.begin(), excludedLmks.end()),
			QSet<qint64>(excludedLcs.begin(), excludedLcs.end())
			};

}

GenericSBAGraphReductor::GenericSBAGraphReductor()
{

}

bool GenericSBAGraphReductor::reduceGraph() {

    int nRemoved;

    do {
        nRemoved = 0;

        QList<qint64> nodesIdxs = _nodes.keys();

        for (qint64 id : nodesIdxs) {
            auto& node = _nodes[id];
            if (node.nObs < node.freedomDegrees) {
                removeItem(id);
                nRemoved++;
            }
        }

    } while (nRemoved > 0);

    return true;

}


bool GenericSBAGraphReductor::isIdConstrained(qint64 id) const {

    if (!_nodes.contains(id)) {
        return false;
    }

    auto& node = _nodes[id];

    return node.freedomDegrees <= node.nObs;
}

bool GenericSBAGraphReductor::insertItem(qint64 id, int freedomDegrees) {

    if (_nodes.contains(id)) {
        return false;
    }

    _nodes.insert(id, node{freedomDegrees, 0});
    return true;
}

bool GenericSBAGraphReductor::insertSelfObservation(qint64 id, int nObs) {

    if (!_nodes.contains(id)) {
        return false;
    }

    _nodes[id].nObs += nObs;
    return true;
}

bool GenericSBAGraphReductor::insertObservation(qint64 id1, qint64 id2, int nObs) {

    if (id1 == id2) {
        return false;
    }

    if (!_nodes.contains(id1) or !_nodes.contains(id2)) {
        return false;
    }

    qint64 min = std::min(id1, id2);
    qint64 max = std::max(id1, id2);

    QPair<qint64, qint64> idx(min, max);

    if (!_observations.contains(idx)) {
        _observations.insert(idx, 0);
        _edges.insert(id1, id2);
        _edges.insert(id2, id1);
    }

    _observations[idx] += nObs;
    _nodes[id1].nObs += nObs;
    _nodes[id2].nObs += nObs;
    return true;

}

bool GenericSBAGraphReductor::removeSelfObservation(qint64 id, int nObs) {
    if (!_nodes.contains(id)) {
        return false;
    }

    auto & node = _nodes[id];

    int externalObs = 0;
    QList<qint64> connections = _edges.values(id);

    for (qint64 cid : connections) {

        qint64 min = std::min(id, cid);
        qint64 max = std::max(id, cid);

        QPair<qint64, qint64> idx(min, max);
        externalObs += _observations[idx];
    }

    if (node.nObs < externalObs + nObs) {
        return false;
    }

    node.nObs -= nObs;
    return true;
}

bool GenericSBAGraphReductor::removeItem(qint64 id) {

    if (!_nodes.contains(id)) {
        return false;
    }

    _nodes.remove(id);
    QList<qint64> connections = _edges.values(id);

    for (qint64 cid : connections) {

        qint64 min = std::min(id, cid);
        qint64 max = std::max(id, cid);

        QPair<qint64, qint64> idx(min, max);

        int nObs = _observations[idx];
        _nodes[cid].nObs -= nObs;
        _observations.remove(idx);
        _edges.remove(cid,id);

    }

    _edges.remove(id);
    return true;

}

void GenericSBAGraphReductor::clear() {
    _nodes.clear();
    _edges.clear();
    _observations.clear();
}

} // namespace StereoVisionApp
