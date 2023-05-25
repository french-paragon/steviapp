#ifndef STEREOVISIONAPP_GEOREFERENCEDDATABLOCKINTERFACE_H
#define STEREOVISIONAPP_GEOREFERENCEDDATABLOCKINTERFACE_H

#include <Eigen/Core>
#include <QString>

#include <QObject>

namespace StereoVisionApp {

/*!
 * \brief The GeoReferencedDataBlockInterface class is an interface for datablocks that can be georeferenced
 *
 * Solvers and Displays needs to put all elements in the same set of coordinates.
 * They also may need to use local coordinates to avoid numerical imprecisions and
 * optimize the display.
 *
 * But the end user working with georeferenced data might want to enter the data
 * using georeferenced
 */
class GeoReferencedDataBlockInterface
{
public:

    /*!
     * \brief The CRSRole enum describe some base roles that can be passed to getCoordinateReferenceSystemDescr
     *
     * This allows a georeferenced datablock to define multiple CRS, which can be queried via a role.
     */
    enum CRSRole {
        DefaultCRSRole = 0,
        ClassDefinedCRSRole = 1
    };

    virtual ~GeoReferencedDataBlockInterface() = default;

    /*!
     * \brief geoReferenceSupportActive indicate if the georeference support is active
     * \return true if the object should be considered to need geo reference support, false otherwise.
     *
     * Some datablocks might be usesable in a georeferenced settings, or in a local reference frame (e.g. landmarks)
     */
    virtual bool geoReferenceSupportActive() const = 0;

    /*!
     * \brief getLocalPointsEcef is used when estimating a local reference system.
     * \return a set of point to consider when building the local coordinates system
     *
     * At some point the project might want to estimate a local frame to use instead of the ECEF frame.
     * To do this, it will query the project for points to consider.
     * This function allows the datablock to sent some points for consideration during this step.
     *
     * The points needs to be already converted to ecef (assumed to be ECEF related to the WGS84 datum, so EPSG:4978)
     */
    virtual Eigen::Array<float,3, Eigen::Dynamic> getLocalPointsEcef() const = 0;

    /*!
     * \brief getCoordinateReferenceSystemDescr describe the coordinate reference system (CRS) used by the datablock
     * \return a string describing the CRS used by the datablock.
     *
     * The description can be in WTK format, can be an identifier of the form "AUTHORITY:CODE" or anything
     * that the OSGeo Proj api can understand.
     *
     * If the georeference support is not active, this function return value is undefined (but a value should be returned).
     */
    virtual QString getCoordinateReferenceSystemDescr(int role = DefaultCRSRole) const = 0;
};

} // namespace StereoVisionApp

#define GeoReferencedDataBlockInterface_iid "org.StereoVisionApp.GeoReferencedDataBlockInterface"

Q_DECLARE_INTERFACE(StereoVisionApp::GeoReferencedDataBlockInterface, GeoReferencedDataBlockInterface_iid)

#endif // STEREOVISIONAPP_GEOREFERENCEDDATABLOCKINTERFACE_H
