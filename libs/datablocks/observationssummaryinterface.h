#ifndef OBSERVATIONSSUMMARYINTERFACE_H
#define OBSERVATIONSSUMMARYINTERFACE_H

#include <QVector>

#include <variant>

#include <QObject>

namespace StereoVisionApp {

/*!
 * \brief The ObservationsSummaryInterface class is an interface that allows to query metadata about how certain datablocks observe other datablocks.
 *
 * This is primarily meant to be used for debuging, to improve certain visualization graphs and related stuff.
 * It could also be used in the future to automatize certain tasks when building factor graphs and other observation models, but is not at the moment.
 *
 */
class ObservationsSummaryInterface
{
public:

    struct Observation {
        QVector<qint64> _observationItemRef; //point to the item encoding the observation, might be a subitem of the item with an ObservationsSummaryInterface
    };

    struct TimedObservation : public Observation {
        double time;
    };

    using GenericObservation = std::variant<Observation, TimedObservation>;

    ObservationsSummaryInterface();

    /*!
     * \brief observationsInfos query information about observation from the datablock to a target
     * \param target the target of the observations
     * \return a list of GenericObservations
     */
    virtual QVector<GenericObservation> observationsInfos(QVector<qint64> const& target) const = 0;
};

} // namespace StereoVisionApp

#define ObservationsSummaryInterface_iid "org.StereoVisionApp.ObservationsSummaryInterface"

Q_DECLARE_INTERFACE(StereoVisionApp::ObservationsSummaryInterface, ObservationsSummaryInterface_iid)

#endif // OBSERVATIONSSUMMARYINTERFACE_H
