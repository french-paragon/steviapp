#ifndef STEREOVISIONAPP_SBAGRAPHREDUCTOR_H
#define STEREOVISIONAPP_SBAGRAPHREDUCTOR_H

#include <QSet>
#include <QMap>
#include <QMultiMap>

namespace StereoVisionApp {

class Project;

class SBAGraphReductor
{
public:
	SBAGraphReductor(int min_img_obs = 3, int min_pt_obs = 2, bool count_self_obs_pos = true, bool count_self_obs_rot = true);

	struct elementsSet {
		QSet<qint64> imgs;
		QSet<qint64> pts;
		QSet<qint64> localcoordsysts;
		QSet<qint64> e_imgs;
		QSet<qint64> e_pts;
		QSet<qint64> e_localcoordsysts;
	};

	elementsSet reduceGraph(Project* p, bool initializedOnly = true) const;
	inline elementsSet operator()(Project* p, bool initializedOnly = true) const {
		return reduceGraph(p, initializedOnly);
	}

private:

	int _min_img_obs;
	int _min_pt_obs;
	bool _count_self_obs_pos;
	bool _count_self_obs_rot;
};

class GenericSBAGraphReductor
{
public:

    GenericSBAGraphReductor();

    /*!
     * \brief reduceGraph remove all unconstrained ids (recusrively) until all elements in the graph are observed
     * \return true on success, false if an error occured
     */
    bool reduceGraph();

    /*!
     * \brief isIdConstrained indicate if an id has more observations than its degrees of freedom
     * \param id
     * \return
     */
    bool isIdConstrained(qint64 id) const;

    bool insertItem(qint64 id, int freedomDegrees);
    bool insertSelfObservation(qint64 id1, int nObs);
    bool insertObservation(qint64 id1, qint64 id2, int nObs);
    bool removeSelfObservation(qint64 id, int nObs);
    bool removeItem(qint64 id);

    void clear();

private:

    struct node {
        int freedomDegrees; //degrees of freedom of the element
        int nObs; //number of observations concerning the element
    };

    QMap<qint64, node> _nodes;
    QMultiMap<qint64, qint64> _edges;
    QMap<QPair<qint64, qint64>, int> _observations;
};

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_SBAGRAPHREDUCTOR_H
