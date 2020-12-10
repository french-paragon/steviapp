#ifndef STEREOVISIONAPP_SBAGRAPHREDUCTOR_H
#define STEREOVISIONAPP_SBAGRAPHREDUCTOR_H

#include <QSet>

namespace StereoVisionApp {

class Project;

class SBAGraphReductor
{
public:
	SBAGraphReductor(int min_img_obs = 3, int min_pt_obs = 2, bool count_self_obs_pos = true, bool count_self_obs_rot = true);

	struct elementsSet {
		QSet<qint64> imgs;
		QSet<qint64> pts;
		QSet<qint64> e_imgs;
		QSet<qint64> e_pts;
	};

	elementsSet reduceGraph(Project* p) const;
	inline elementsSet operator()(Project* p) const {
		return reduceGraph(p);
	}

private:

	int _min_img_obs;
	int _min_pt_obs;
	bool _count_self_obs_pos;
	bool _count_self_obs_rot;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_SBAGRAPHREDUCTOR_H
