
#include <CoverageControl/parameters.h>


namespace CoverageControl {
	void Parameters::ParseConfig() {
		std::cout << "Using config file: " << config_file_ << std::endl;
		/* if(not std::filesystem::exists(config_file_)) { */
		/* 	std::cerr << "Could not find config file " << config_file_ << std::endl; */
		/* 	throw std::runtime_error("Could not open config file"); */
		/* } */
		YAML::Node yaml_config_ = YAML::LoadFile(config_file_);
		pResolution = yaml_config_["pResolution"].as<double>();
		pWorldMapSize = yaml_config_["pWorldMapSize"].as<int>();
		pRobotMapSize = yaml_config_["pRobotMapSize"].as<int>();
		pUnknownImportance = yaml_config_["pUnknownImportance"].as<double>();
		pRobotMapUseUnknownImportance = yaml_config_["pRobotMapUseUnknownImportance"].as<bool>();
		pLocalMapSize = yaml_config_["pLocalMapSize"].as<int>();
		pCommunicationRange = yaml_config_["pCommunicationRange"].as<double>();
		pRobotInitDist = yaml_config_["pRobotInitDist"].as<double>();
		pUpdateRobotMap = yaml_config_["pUpdateRobotMap"].as<bool>();
		pUpdateExplorationMap = yaml_config_["pUpdateExplorationMap"].as<bool>();
		pUpdateSensorView = yaml_config_["pUpdateSensorView"].as<bool>();
		pSensorSize = yaml_config_["pSensorSize"].as<int>();
		pTimeStep = yaml_config_["pTimeStep"].as<double>();
		pMaxRobotSpeed = yaml_config_["pMaxRobotSpeed"].as<double>();
		pEpisodeSteps = yaml_config_["pEpisodeSteps"].as<int>();
		pTruncationBND = yaml_config_["pTruncationBND"].as<double>();
		pNorm = yaml_config_["pNorm"].as<double>();
		pMinSigma = yaml_config_["pMinSigma"].as<double>();
		pMaxSigma = yaml_config_["pMaxSigma"].as<double>();
		pMinPeak = yaml_config_["pMinPeak"].as<double>();
		pMaxPeak = yaml_config_["pMaxPeak"].as<double>();
		pLloydMaxIterations = yaml_config_["pLloydMaxIterations"].as<int>();
		pLloydNumTries = yaml_config_["pLloydNumTries"].as<int>();
	}
} /* namespace CoverageControl */
