#include <CoverageControl/coverage_system.h>
#include <CoverageControl/plotter.h>


namespace CoverageControl {

	void CoverageSystem::PlotSystemMap(std::string const &dir_name, int const &step, std::vector<int> const &robot_status) const {
		Plotter<MapType> plotter(dir_name, params_.pWorldMapSize * params_.pResolution, params_.pResolution);
		plotter.SetPlotName("map", step);
		plotter.PlotMap(system_map_, robot_global_positions_, robot_positions_history_, robot_status);
	}

	void CoverageSystem::PlotWorldMap(std::string const &dir_name) const {
		Plotter<MapType> plotter(dir_name, params_.pWorldMapSize * params_.pResolution, params_.pResolution);
		plotter.SetPlotName("WorldMap");
		plotter.PlotMap(GetWorldIDF());
	}

	void CoverageSystem::PlotMapVoronoi(std::string const &dir_name, int const &step, Voronoi const &voronoi, PointVector const &goals) const {
		Plotter<MapType> plotter(dir_name, params_.pWorldMapSize * params_.pResolution, params_.pResolution);
		plotter.SetPlotName("map", step);
		plotter.PlotMap(GetWorldIDF(), robot_global_positions_, goals, voronoi);
	}

	void CoverageSystem::PlotFrontiers(std::string const &dir_name, int const &step, PointVector const &frontiers) const {
		Plotter<MapType> plotter(dir_name, params_.pWorldMapSize * params_.pResolution, params_.pResolution);
		plotter.SetPlotName("map", step);
		plotter.PlotMap(GetWorldIDF(), robot_global_positions_, frontiers);
	}

	void CoverageSystem::PlotRobotSystemMap(std::string const &dir_name, int const &robot_id, int const &step) {
		Plotter<MapType> plotter(dir_name, params_.pLocalMapSize * params_.pResolution, params_.pResolution);
		plotter.SetPlotName("robot_" + std::to_string(robot_id) + "_", step);
		plotter.PlotMap(GetRobotSystemMap(robot_id));
	}

	void CoverageSystem::PlotRobotIDFMap(std::string const &dir_name, int const &robot_id, int const &step) {
		Plotter<MapType> plotter(dir_name, params_.pLocalMapSize * params_.pResolution, params_.pResolution);
		plotter.SetPlotName("robot_" + std::to_string(robot_id) + "_", step);
		plotter.PlotMap(GetRobotLocalMap(robot_id));
	}

	void CoverageSystem::PlotRobotExplorationMap(std::string const &dir_name, int const &robot_id, int const &step) {
		Plotter<MapType> plotter(dir_name, params_.pLocalMapSize * params_.pResolution, params_.pResolution);
		plotter.SetPlotName("robot_exp_" + std::to_string(robot_id) + "_", step);
		plotter.PlotMap(GetRobotExplorationMap(robot_id));
	}

	void CoverageSystem::PlotRobotSensorView(std::string const &dir_name, int const &robot_id, int const &step) {
		Plotter<MapType> plotter(dir_name, params_.pLocalMapSize * params_.pResolution, params_.pResolution);
		plotter.SetPlotName("robot_sensor_" + std::to_string(robot_id) + "_", step);
		plotter.PlotMap(GetRobotSensorView(robot_id));
	}
}	// namespace CoverageControl
