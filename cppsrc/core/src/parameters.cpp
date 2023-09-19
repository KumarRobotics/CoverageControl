#include "../include/CoverageControl/parameters.h"
#include <yaml-cpp/yaml.h>


namespace CoverageControl {
	void Parameters::ParseConfig() {
		std::cout << "Using config file: " << config_file_ << std::endl;
		if(not std::filesystem::exists(config_file_)) {
			std::cerr << "Could not find config file " << config_file_ << std::endl;
			throw std::runtime_error("Could not open config file");
		}

		YAML::Node yaml_config_ = YAML::LoadFile(config_file_);

		pNumRobots = yaml_config_["pNumRobots"].as<int>();
		pNumFeatures = yaml_config_["pNumFeatures"].as<int>();

		auto env_maps_yaml = yaml_config_["EnvMaps"];

		pResolution = env_maps_yaml["pResolution"].as<double>();
		pWorldMapSize = env_maps_yaml["pWorldMapSize"].as<int>();
		pRobotMapSize = env_maps_yaml["pRobotMapSize"].as<int>();
		pLocalMapSize = env_maps_yaml["pLocalMapSize"].as<int>();

		auto env_idf_yaml = yaml_config_["EnvIDF"];

		pTruncationBND = env_idf_yaml["pTruncationBND"].as<double>();
		pNorm = env_idf_yaml["pNorm"].as<double>();
		pMinSigma = env_idf_yaml["pMinSigma"].as<double>();
		pMaxSigma = env_idf_yaml["pMaxSigma"].as<double>();
		pMinPeak = env_idf_yaml["pMinPeak"].as<double>();
		pMaxPeak = env_idf_yaml["pMaxPeak"].as<double>();
		pUnknownImportance = env_idf_yaml["pUnknownImportance"].as<double>();
		pRobotMapUseUnknownImportance = env_idf_yaml["pRobotMapUseUnknownImportance"].as<bool>();

		auto map_updates_yaml = yaml_config_["MapUpdates"];

		pUpdateRobotMap = map_updates_yaml["pUpdateRobotMap"].as<bool>();
		pUpdateSensorView = map_updates_yaml["pUpdateSensorView"].as<bool>();
		pUpdateExplorationMap = map_updates_yaml["pUpdateExplorationMap"].as<bool>();
		pUpdateSystemMap = map_updates_yaml["pUpdateSystemMap"].as<bool>();

		auto robot_model_yaml = yaml_config_["RobotModel"];

		pSensorSize = robot_model_yaml["pSensorSize"].as<int>();
		pCommunicationRange = robot_model_yaml["pCommunicationRange"].as<double>();
		pMaxRobotSpeed = robot_model_yaml["pMaxRobotSpeed"].as<double>();
		pRobotInitDist = robot_model_yaml["pRobotInitDist"].as<double>();
		pRobotPosHistorySize = robot_model_yaml["pRobotPosHistorySize"].as<int>();
		pTimeStep = robot_model_yaml["pTimeStep"].as<double>();

		if (yaml_config_["Algorithm"]) {
			auto algorithm_yaml = yaml_config_["Algorithm"];
			pEpisodeSteps = algorithm_yaml["pEpisodeSteps"].as<int>();
			pLloydMaxIterations = algorithm_yaml["pLloydMaxIterations"].as<int>();
			pLloydNumTries = algorithm_yaml["pLloydNumTries"].as<int>();
			pNumFrontiers = algorithm_yaml["pNumFrontiers"].as<int>();
		}

		if (robot_model_yaml["pAddNoisePositions"]) {
			pAddNoisePositions = robot_model_yaml["pAddNoisePositions"].as<bool>();
			pPositionsNoiseSigma = robot_model_yaml["pPositionsNoiseSigma"].as<double>();
		}

	}
} /* namespace CoverageControl */
