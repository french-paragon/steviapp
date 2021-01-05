#include "sbagraphreductor.h"

#include "datablocks/project.h"
#include "datablocks/image.h"
#include "datablocks/landmark.h"

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

	QSet<qint64> imgs(imgs_v.begin(), imgs_v.end());
	QSet<qint64> lmks(lmks_v.begin(), lmks_v.end());

	QVector<qint64> excludedImgs;
	QVector<qint64> excludedLmks;

	excludedImgs.reserve(imgs.size());
	excludedLmks.reserve(imgs.size());

	int eImgsPos = 0;
	int eLmksPos = 0;

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
					connections++;
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

			if (connections < _min_img_obs) {
				excludedImgs.push_back(id);
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

			if (_count_self_obs_pos) {
				if (lm->xCoord().isSet() or lm->yCoord().isSet() or lm->zCoord().isSet()) {
					connections++;
				}
			}

			if (connections < _min_pt_obs) {
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

	}

	return {imgs,
			lmks,
			QSet<qint64>(excludedImgs.begin(), excludedImgs.end()),
			QSet<qint64>(excludedLmks.begin(), excludedLmks.end())
			};

}

} // namespace StereoVisionApp
