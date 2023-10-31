#include "../include/CoverageControl/coverage_system.h"
#include "../include/CoverageControl/plotter.h"
#include <thread>


namespace CoverageControl {

	void CoverageSystem::RenderRecordedMap(std::string const &dir_name, std::string const &video_name) const {
		std::string frame_dir = dir_name + "/frames/";
		std::filesystem::create_directory(frame_dir);
		Plotter plotter(frame_dir, params_.pWorldMapSize * params_.pResolution, params_.pResolution);
		plotter.SetScale(2);
#pragma omp parallel for num_threads(std::thread::hardware_concurrency()/2)
		for(size_t i = 0; i < plotter_data_.size(); ++i) {
			auto iPlotter = plotter;
			iPlotter.SetPlotName("map", i);
			iPlotter.PlotMap(plotter_data_[i].map, plotter_data_[i].positions, plotter_data_[i].positions_history, plotter_data_[i].robot_status);
		}
		system(("ffmpeg -y -r 30 -i " + frame_dir + "map%04d.png -vcodec libx264 -crf 25  -pix_fmt yuv420p " + dir_name + "/" + video_name).c_str());
		std::filesystem::remove_all(frame_dir);
	}

	void CoverageSystem::RecordPlotData(std::vector <int> const &robot_status, std::string const &map_name) {
		PlotterData data;
		if(map_name == "world") {
			data.map = GetWorldIDF();
		} else {
			data.map = system_map_;
		}
		data.positions = robot_global_positions_;
		data.positions_history = robot_positions_history_;
		data.robot_status = robot_status;
		plotter_data_.push_back(data);
	}

	void CoverageSystem::PlotSystemMap(std::string const &dir_name, int const &step, std::vector<int> const &robot_status) const {
		Plotter plotter(dir_name, params_.pWorldMapSize * params_.pResolution, params_.pResolution);
		plotter.SetScale(1);
		plotter.SetPlotName("map", step);
		plotter.PlotMap(system_map_, robot_global_positions_, robot_positions_history_, robot_status);
	}

	void CoverageSystem::PlotWorldMapRobots(std::string const &dir_name, std::string const &map_name) const {
		Plotter plotter(dir_name, params_.pWorldMapSize * params_.pResolution, params_.pResolution);
		plotter.SetScale(1);
		plotter.SetPlotName(map_name);
		std::vector<int> robot_status(num_robots_, 0);
		plotter.PlotMap(GetWorldIDF(), robot_global_positions_, robot_positions_history_, robot_status);
	}

	void CoverageSystem::PlotWorldMap(std::string const &dir_name, std::string const &map_name) const {
		Plotter plotter(dir_name, params_.pWorldMapSize * params_.pResolution, params_.pResolution);
		plotter.SetScale(1);
		plotter.SetPlotName(map_name);
		plotter.PlotMap(GetWorldIDF());
	}

	void CoverageSystem::PlotInitMap(std::string const &dir_name, std::string const &map_name) const {
		Plotter plotter(dir_name, params_.pWorldMapSize * params_.pResolution, params_.pResolution);
		plotter.SetScale(2);
		plotter.SetPlotName(map_name);
		plotter.PlotMap(GetWorldIDF(), robot_global_positions_);
	}

	void CoverageSystem::PlotMapVoronoi(std::string const &dir_name, int const &step) {
		ComputeVoronoiCells();
		Plotter plotter(dir_name, params_.pWorldMapSize * params_.pResolution, params_.pResolution);
		plotter.SetScale(2);
		plotter.SetPlotName("voronoi_map", step);
		plotter.PlotMap(GetWorldIDF(), robot_global_positions_, voronoi_, robot_positions_history_);
	}

	void CoverageSystem::PlotMapVoronoi(std::string const &dir_name, int const &step, Voronoi const &voronoi, PointVector const &goals) const {
		Plotter plotter(dir_name, params_.pWorldMapSize * params_.pResolution, params_.pResolution);
		plotter.SetScale(2);
		plotter.SetPlotName("map", step);
		plotter.PlotMap(GetWorldIDF(), robot_global_positions_, goals, voronoi);
	}

	void CoverageSystem::PlotFrontiers(std::string const &dir_name, int const &step, PointVector const &frontiers) const {
		Plotter plotter(dir_name, params_.pWorldMapSize * params_.pResolution, params_.pResolution);
		plotter.SetScale(2);
		plotter.SetPlotName("map", step);
		plotter.PlotMap(system_map_, robot_global_positions_, robot_positions_history_, frontiers);
	}

	void CoverageSystem::PlotRobotSystemMap(std::string const &dir_name, int const &robot_id, int const &step) {
		Plotter plotter(dir_name, params_.pLocalMapSize * params_.pResolution, params_.pResolution);
		plotter.SetPlotName("robot_" + std::to_string(robot_id) + "_", step);
		PointVector neighbours_positions = GetRelativePositonsNeighbors(robot_id);
		for(Point2 &pos : neighbours_positions) {
			pos[0] += params_.pLocalMapSize / 2;
			pos[1] += params_.pLocalMapSize / 2;
		}
		plotter.PlotMap(GetRobotSystemMap(robot_id), neighbours_positions);
	}

	void CoverageSystem::PlotRobotIDFMap(std::string const &dir_name, int const &robot_id, int const &step) {
		Plotter plotter(dir_name, params_.pLocalMapSize * params_.pResolution, params_.pResolution);
		plotter.SetScale(4);
		plotter.SetPlotName("robot_" + std::to_string(robot_id) + "_", step);
		plotter.PlotMap(GetRobotLocalMap(robot_id));
	}

	void CoverageSystem::PlotRobotExplorationMap(std::string const &dir_name, int const &robot_id, int const &step) {
		Plotter plotter(dir_name, params_.pLocalMapSize * params_.pResolution, params_.pResolution);
		plotter.SetPlotName("robot_exp_" + std::to_string(robot_id) + "_", step);
		plotter.PlotMap(GetRobotExplorationMap(robot_id));
	}

	void CoverageSystem::PlotRobotSensorView(std::string const &dir_name, int const &robot_id, int const &step) {
		Plotter plotter(dir_name, params_.pSensorSize * params_.pResolution, params_.pResolution);
		plotter.SetPlotName("robot_sensor_" + std::to_string(robot_id) + "_", step);
		plotter.PlotMap(GetRobotSensorView(robot_id));
	}

	void CoverageSystem::PlotRobotLocalMap(std::string const &dir_name, int const &robot_id, int const &step) {
		Plotter plotter(dir_name, params_.pLocalMapSize * params_.pResolution, params_.pResolution);
		plotter.SetPlotName("robot_map_" + std::to_string(robot_id) + "_", step);
		plotter.PlotMap(GetRobotLocalMap(robot_id));
	}

	void CoverageSystem::PlotRobotObstacleMap(std::string const &dir_name, int const &robot_id, int const &step) {
		Plotter plotter(dir_name, params_.pLocalMapSize * params_.pResolution, params_.pResolution);
		plotter.SetPlotName("robot_obstacle_map_" + std::to_string(robot_id) + "_", step);
		plotter.PlotMap(GetRobotObstacleMap(robot_id));
	}

	void CoverageSystem::PlotRobotCommunicationMaps(std::string const &dir_name, int const &robot_id, int const &step, size_t const &map_size) {
		auto robot_communication_maps = GetCommunicationMap(robot_id, map_size);
		Plotter plotter_x(dir_name, map_size * params_.pResolution, params_.pResolution);
		plotter_x.SetPlotName("robot_communication_map_x_" + std::to_string(robot_id) + "_", step);
		plotter_x.PlotMap(robot_communication_maps.first);
		Plotter plotter_y(dir_name, map_size * params_.pResolution, params_.pResolution);
		plotter_y.SetPlotName("robot_communication_map_y_" + std::to_string(robot_id) + "_", step);
		plotter_y.PlotMap(robot_communication_maps.second);
	}

}	// namespace CoverageControl
