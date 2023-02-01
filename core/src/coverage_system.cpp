#include <CoverageControl/coverage_system.h>
#include <gnuplot/gnuplot-iostream.h>

#include <filesystem>

namespace CoverageControl {

	void CoverageSystem::PlotSystemMap(std::string const &dir_name, int step, std::vector<int> robot_status) const {
		if(robot_status.size() == 0) {
			robot_status = std::vector<int>(num_robots_, 0);
		}
		std::filesystem::path path{dir_name};
		if(not std::filesystem::exists(path)) {
			std::cerr << "Directory does not exist" << std::endl;
			throw std::runtime_error{"Directory does not exist"};
		}

		std::stringstream ss;
		ss << std::setw(4) << std::setfill('0') << step;
		std::string s = ss.str();
		std::string imap_name = "map" + s;
		std::string map_filename {std::filesystem::absolute(path/imap_name)};
		std::cout << map_filename << std::endl;

		std::string data_filename {std::filesystem::absolute(path/"data")};
		std::string pos_filename {std::filesystem::absolute(path/"pos")};

		MapUtils::WriteMap(system_map_, data_filename);
		WriteRobotPositions(pos_filename);

		for(size_t iRobot = 0; iRobot < num_robots_; ++iRobot) {
			std::ofstream file_obj(pos_filename + std::to_string(iRobot));
			for(auto const &pos : robot_positions_history_[iRobot]) {
				file_obj << pos[0] << " " << pos[1] << std::endl;
			}
			file_obj.close();
		}

		Gnuplot gp;
		gp << "set terminal pngcairo enhanced font 'Times,28' size 2048, 2048\n";
		gp << "set o '" << map_filename << ".png'\n";
		gp << "set palette defined (-1 '#aeb6bf', 0 'white', 1 '#900C3F')\n";
		gp << "set xrange [0:" << params_.pWorldMapSize * params_.pResolution << "]\n";
		gp << "set yrange [0:" << params_.pWorldMapSize * params_.pResolution << "]\n";
		gp << "set cbrange [-1:1]\n";
		gp << "set size ratio -1\n";
		gp << "unset colorbox\n";
		std::string res = std::to_string(params_.pResolution);
		gp << "plot '"<< data_filename << "' matrix using ($2*" << res << "):($1*" << res << "):3 with image notitle ";
		for(size_t iRobot = 0; iRobot < num_robots_; ++iRobot) {
			if(robot_status[iRobot] == 0) {
				gp << ", '" << pos_filename << std::to_string(iRobot) << "' with line lw 4 lc rgb '#1b4f72' notitle";
			} else {
				gp << ", '" << pos_filename << std::to_string(iRobot) << "' with line lw 4 lc rgb '#1b4f72' notitle";
			}
		}
		for(size_t iRobot = 0; iRobot < num_robots_; ++iRobot) {
			if(robot_status[iRobot] == 0) {
				gp << ",'-' with points pt 7 ps 4 lc rgb '#1b4f72' notitle";
			} else {
				gp << ",'-' with points pt 7 ps 4 lc rgb '#1b4f72' notitle";
				/* gp << ",'-' with points pt 7 ps 2 lc rgb '#196f3d' notitle"; */
			}
		}
		gp << "\n";
		for(size_t iRobot = 0; iRobot < num_robots_; ++iRobot) {
			gp << robot_global_positions_[iRobot][0] << " " << robot_global_positions_[iRobot][1] << std::endl;
			gp << "e\n";;
		}

	}

	void CoverageSystem::PlotWorldMap(std::string const &dir_name) const {
		std::filesystem::path path{dir_name};
		if(not std::filesystem::exists(path)) {
			std::cerr << "Directory does not exist" << std::endl;
			throw std::runtime_error{"Directory does not exist"};
		}

		std::string imap_name = "WorldMap";
		std::string map_filename {std::filesystem::absolute(path/imap_name)};
		std::cout << map_filename << std::endl;

		std::string data_filename {std::filesystem::absolute(path/"data")};
		std::string pos_filename {std::filesystem::absolute(path/"pos")};

		MapUtils::WriteMap(GetWorldIDF(), data_filename);
		WriteRobotPositions(pos_filename);

		Gnuplot gp;
		gp << "set terminal pngcairo enhanced font 'Times,28' size 2048, 2048\n";
		gp << "set o '" << map_filename << ".png'\n";
		gp << "set palette defined (-1 '#aeb6bf', 0 'white', 1 '#900C3F')\n";
		gp << "set xrange [0:" << params_.pWorldMapSize * params_.pResolution << "]\n";
		gp << "set yrange [0:" << params_.pWorldMapSize * params_.pResolution << "]\n";
		gp << "set cbrange [-1:1]\n";
		gp << "set size ratio -1\n";
		gp << "unset colorbox\n";
		std::string res = std::to_string(params_.pResolution);
		gp << "plot '"<< data_filename << "' matrix using ($2*" << res << "):($1*" << res << "):3 with image notitle ";
		gp << ",'" << pos_filename << "' with points pt 7 ps 4 lc rgb '#1b4f72' notitle";
		gp << "\n";
	}
}	// namespace CoverageControl
