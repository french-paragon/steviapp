#ifndef STANDARDPROJECTGENERATORS_H
#define STANDARDPROJECTGENERATORS_H

#include <string>

namespace StereoVisionApp {

class Project;

/*!
 * \brief the StandardProjectGenerators namespace contain standard generator to get procedural tests projects
 *
 * each functions has signature bool generator(Project* p, int seed = 42, [faculative parameter arguments]);
 *
 * It is assumed that the project has been configured with the proper classes factories, if not, building will fail and the generator will return false.
 */
namespace StandardProjectGenerators
{

/*!
 * \brief simplePnPGenerator generate a simple PnP problem in the project
 * \param p the project
 * \param seed the random seed
 * \param nImages the number of images to generate in the project
 * \return true if sucess, false otherwise
 *
 * The generator will add images, a camera and a correspondance set and/landmarks to the project
 */
bool simplePnPGenerator(Project* p,
                        int seed = 42,
                        int nImages = 1,
                        bool withCorrespondance = true,
                        bool fixedPoints = true);


/*!
 * \brief circularTrajectoryEcef generate a trajectory in the project, representing a circular trajectory in ECEF
 * \param p the project the trajectory should be generated into.
 * \param seed the random seed
 * \param duration the duration of the full circle, in seconds
 * \param accDt the sampling time for the accelerometer/gyroscope
 * \param posDt the sampling time for the gps
 * \param name the name that will be given to the trajectory
 * \return true if sucess, false otherwise
 *
 * The generator will add a trajectory to the project.
 * It is assumed that the factory registered for trajectory will create a generated trajectory.
 */
bool circularTrajectoryEcef(Project* p,
                            int seed = 42,
                            float duration = 100,
                            float samplingDt = 0.5,
                            float accDt = 0.05,
                            float posDt = 5.0,
                            std::string const& name = "simulated_circle");


} // namespace StandardProjectGenerators

} // namespace StereoVisionApp

#endif // STANDARDPROJECTGENERATORS_H
