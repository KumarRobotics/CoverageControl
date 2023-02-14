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

#include <filesystem>
#include <iostream>
#include <cmath>
#include <yaml-cpp/yaml.h>

namespace CoverageControl {

	class Parameters {
		public:

			std::string config_file_;

			// Assuming same resolution in both the directions. Pixel area = pResolution^2
			double pResolution = 1;

			// Actual size of maps is size * pResolution, e.g.,  pWorldMapSize * pResolution
			// For ~1 km^2 actual environment, we have a 1024 grid world with each cell pResolution * pRobotMapSize=1 m^2
			int pWorldMapSize = 1024;

			// Robot map saves what the robot has seen
			// Could make it sparse if size becomes a problem
			int pRobotMapSize = pWorldMapSize;
			double pUnknownImportance = 0.5;
			bool pRobotMapUseUnknownImportance = false;

			// Local map is used for computing mass. Actual area would be pLocalMapSize * pResolution
			// Should be greater than pCommunicationRange so that they can form different channels of the same image.
			int pLocalMapSize = 256;
			double pCommunicationRange = 256; // Radius of communication (in meters)
			double pRobotInitDist = 1024; // Distance from the origin within which to initialize the position of the robots
			int pRobotPosHistorySize = 50; // Number of previous positions to store

			// Set pUpdateRobotMap to false for centralized known world
			bool pUpdateRobotMap = true;
			bool pUpdateExplorationMap = true;
			bool pUpdateSensorView = true;

			bool pUpdateSystemMap = true;

			// Assuming square sensor FOV.
			// Actual FOV: square with side pResolution * pSensorSize
			// Robot is placed at the center of FOV
			// Make it even so that I don't have to deal with substracting by half-resolution.
			// Have made it to half of (pWorldMapSize - 1000 / pResolution)/2
			int pSensorSize = 32; // Positive integer. NOTE: Needs to be even

			// Each time step corresponds to 0.1s, i.e., 10Hz
			double pTimeStep = 1;
			// in m/s. Make sure pMaxRobotSpeed * pTimeStep / pResolution < pSensorSize/2
			double pMaxRobotSpeed = 5;
			// This is not cause a hard constraint, but helpful for initializing vectors

			int pEpisodeSteps = 2000; // Total time is pEpisodeSteps * pTimeStep

			// Bivariate Normal Distribution truncated after pTruncationBND * sigma
			// Helps in reducing the number of erfc evaluations
			// Needs testing to be sure that the probability masses are not significantly off
			double pTruncationBND = 2;

			// Used to normalize map. Max value will be scaled to pNorm. 255 for images
			double pNorm = 1;

			// These settings are only required if the IDF is generated using random gaussians
			double pMinSigma = 40;
			double pMaxSigma = 50;
			double pMinPeak = 6;
			double pMaxPeak = 10;

			int pLloydMaxIterations = 100;
			int pLloydNumTries = 10;

			Parameters() {}

			Parameters (std::string const &config_file) : config_file_{config_file}{
				/* if(not std::filesystem::exists(config_file_)) { */
				/* 	std::cerr << config_file_ << std::endl; */
				/* 	throw std::runtime_error{"Config file not found"}; */
				/* } */
				ParseConfig();
			}

			void SetConfig (std::string const &config_file) {
				config_file_ = config_file;
				/* if(not std::filesystem::exists(config_file_)) { */
				/* 	std::cerr << config_file_ << std::endl; */
				/* 	throw std::runtime_error{"Config file not found"}; */
				/* } */
				ParseConfig();
			}
			void ParseConfig();

	};

} /* namespace CoverageControl */

#endif /* COVERAGECONTROL_PARAMETERS_H_ */

