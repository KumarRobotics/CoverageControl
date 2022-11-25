/**
 * Contains parameters. These are const and variable names start with lower-case p and use CamelCase
 * Primarily used by the RobotModel class, see robot_model.h
 *
 * Generally parameters are set during runtime through configuration files.
 * However, that may cause decrease in speed.
 * To favor highly efficient data generation, I have added the parameters as a header file, which is used during compilation time.
 *
 * Cons: requires compilation whenever there is a change in the model.
 *
 **/

#ifndef COVERAGECONTROL_PARAMETERS_H_
#define COVERAGECONTROL_PARAMETERS_H_

#include <cmath>

namespace CoverageControl {

	// Assuming same resolution in both the directions. Pixel area = pResolution^2
	double const pResolution = 1;

	// Actual size of maps is size * pResolution, e.g.,  pWorldMapSize * pResolution
	// For ~1 km^2 actual environment, we would have a 512 grid world with each cell representing pRobotMapSize=2 m^2
	int constexpr pWorldMapSize = 1024;

	// Robot map saves what the robot has seen
	// Could make it sparse if size becomes a problem
	int constexpr pRobotMapSize = pWorldMapSize;
	double constexpr pUnknownImportance = 0.01;

	// Local map is used for computing mass. Actual area would be pLocalMapSize * pResolution
	// Should be greater than pCommunicationRange so that they can form different channels of the same image.
	int const pLocalMapSize = 64;
	double const pCommunicationRange = 64; // Radius of communication

	// Assuming square sensor FOV.
	// Actual FOV: square with side pResolution * pSensorSize
	// Robot is placed at the center of FOV
	// Make it even so that I don't have to deal with substracting by half-resolution.
	// Have made it to half of (pWorldMapSize - 1000 / pResolution)/2
	int const pSensorSize = 12; // Positive integer. NOTE: Needs to be even

	// Each time step corresponds to 0.1s, i.e., 10Hz
	double const pTimeStep = 0.1;
	// in m/s. Make sure pMaxRobotSpeed * pTimeStep / pResolution < pSensorSize/2
	double const pMaxRobotSpeed = 5;
	// This is not cause a hard constraint, but helpful for initializing vectors

	double const pEpisodeSteps = 1000; // Total time is pEpisodeSteps * pTimeStep

	// Bivariate Normal Distribution truncated after pTruncationBND * sigma
	// Helps in reducing the number of erfc evaluations
	// Needs testing to be sure that the probability masses are not significantly off
	double const pTruncationBND = 4;

	// These settings are only required if the IDF is generated using random gaussians
	double const pMinVariance = 1;
	double const pMaxVariance = 10;
	double const pMinPeak = 5;
	double const pMaxPeak = 10;

} /* namespace CoverageControl */

#endif /* COVERAGECONTROL_PARAMETERS_H_ */

