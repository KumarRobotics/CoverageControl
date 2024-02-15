/*!
 * This file is part of the CoverageControl library.
 *
 * Contains parameters. These are const and variable names start with lower-case p and use CamelCase
 * Primarily used by the RobotModel class, see robot_model.h
 *
 * Generally parameters are set during runtime through configuration files.
 * However, that may cause decrease in speed.
 * To favor highly efficient data generation, I have added the parameters as a header file, which is used during compilation time.
 *
 * Cons: requires compilation whenever there is a change in the model.
 *
 * TODO:
 *
 * @author Saurav Agarwal
 * @contact sauravag@seas.upenn.edu, agr.saurav1@gmail.com
 * Repository: https://github.com/KumarRobotics/CoverageControl
 *
 * The CoverageControl library is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
 *
 * DISCLAIMER OF WARRANTIES: THE SOFTWARE IS PROVIDED "AS-IS" WITHOUT WARRANTY OF ANY KIND INCLUDING ANY WARRANTIES OF PERFORMANCE OR MERCHANTABILITY OR FITNESS FOR A PARTICULAR USE OR PURPOSE OR OF NON-INFRINGEMENT. YOU BEAR ALL RISK RELATING TO QUALITY AND PERFORMANCE OF THE SOFTWARE OR HARDWARE.
 *
 * SUPPORT AND MAINTENANCE: No support, installation, or training is provided.
 *
 * You should have received a copy of the GNU General Public License along with CoverageControl library. If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef COVERAGECONTROL_PARAMETERS_H_
#define COVERAGECONTROL_PARAMETERS_H_

#include <filesystem>
#include <iostream>
#include <cmath>

namespace CoverageControl {

	/*!
	 * \brief Class to store parameters
	 * \details The parameters are for the robot model and the environment.
	 * The parameters are set during runtime through configuration files.
	 */
	class Parameters {
		public:

			std::string config_file_;

			int pNumRobots = 10; //!< Number of robots
			int pNumFeatures = 5; //!< Number of features

			//! Assuming same resolution in both the directions in meters. Pixel area = pResolution^2
			double pResolution = 1.0;

			//! Actual size of maps is size * pResolution, e.g.,  pWorldMapSize * pResolution
			int pWorldMapSize = 1024;

			//! Robot map saves what the robot has seen
			int pRobotMapSize = pWorldMapSize;

			//! Local map is used for computing mass. Actual area would be pLocalMapSize * pResolution
			//!\warning Should be greater than pCommunicationRange so that they can form different channels of the same image.
			int pLocalMapSize = 256;

			bool pUpdateRobotMap = true;
			bool pUpdateExplorationMap = true;
			bool pUpdateSensorView = true;
			bool pUpdateSystemMap = true;

			//! Bivariate Normal Distribution truncated after pTruncationBND * sigma
			// Helps in reducing the number of erfc evaluations
			// Needs testing to be sure that the probability masses are not significantly off
			double pTruncationBND = 2;

			double pNorm = 1;

			// These settings are only required if the IDF is generated using random gaussians
			double pMinSigma = 40;
			double pMaxSigma = 50;
			double pMinPeak = 6;
			double pMaxPeak = 10;

			double pUnknownImportance = 0.5;
			bool pRobotMapUseUnknownImportance = false;

			// Assuming square sensor FOV.
			// Actual FOV: square with side pResolution * pSensorSize
			// Robot is placed at the center of FOV
			// Make it even so that I don't have to deal with substracting by half-resolution.
			// Have made it to half of (pWorldMapSize - 1000 / pResolution)/2
			int pSensorSize = 32; //! \warning Positive integer. NOTE: Needs to be even
			double pCommunicationRange = 256; //!< Radius of communication (in meters)
			// in m/s. Make sure pMaxRobotSpeed * pTimeStep / pResolution < pSensorSize/2
			double pMaxRobotSpeed = 5;
			double pRobotInitDist = 1024; //!< Distance from the origin within which to initialize the position of the robots
			int pRobotPosHistorySize = 20; //!< Number of previous positions to store
			double pTimeStep = 1; //!< Each time step corresponds to pTimeStep seconds
			bool pAddNoisePositions = false;
			double pPositionsNoiseSigma = 0.;

			int pEpisodeSteps = 2000; // Total time is pEpisodeSteps * pTimeStep

			int pLloydMaxIterations = 100;
			int pLloydNumTries = 10;

			int pNumFrontiers = 10; // Number of frontiers to be selected

			Parameters() {}

			Parameters (std::string const &config_file) : config_file_{config_file}{
				ParseParameters();
				PrintParameters();
			}

			void SetConfig (std::string const &config_file) {
				config_file_ = config_file;
				ParseParameters();
			}

			void PrintParameters();

		private:
			void ParseParameters();

	};

} /* namespace CoverageControl */

#endif /* COVERAGECONTROL_PARAMETERS_H_ */

