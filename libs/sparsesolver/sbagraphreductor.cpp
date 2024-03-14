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

} // namespace StereoVisionApp
