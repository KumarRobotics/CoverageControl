#include <CoverageControl/coverage_system.h>
#include <gnuplot/gnuplot-iostream.h>

#include <filesystem>

namespace CoverageControl {

	void GnuplotCommands(Gnuplot &gp, double const &max, std::string &marker_sz, double scale = 1, bool unset_colorbox = true) {
		marker_sz = std::to_string(2 * scale);
		std::string size = std::to_string(1024 * scale);
		std::string font = std::to_string(14 * scale);
		gp << "set terminal pngcairo enhanced font 'Times," << font << "' size " << size << "," << size <<"\n";
		gp << "set palette defined (-1 '#aeb6bf', 0 'white', 1 '#900C3F')\n";
		gp << "set cbrange [-1:1]\n";
		gp << "set size ratio -1\n";
		gp << "set xrange [0:" << std::to_string(max) << "]\n";
		gp << "set yrange [0:" << std::to_string(max) << "]\n";
		if(unset_colorbox)
			gp << "unset colorbox\n";
	}

	void CoverageSystem::PlotSystemMap(std::string const &dir_name, int const &step, std::vector<int> const &robot_status) const {
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

		Gnuplot gp;
		std::string marker_sz;
		GnuplotCommands(gp, params_.pWorldMapSize * params_.pResolution, marker_sz);

		gp << "set o '" << map_filename << ".png'\n";

		std::string res = std::to_string(params_.pResolution);
		gp << "plot '-' matrix using ($2*" << res << "):($1*" << res << "):3 with image notitle";
		for(size_t iRobot = 0; iRobot < num_robots_; ++iRobot) {
			if(robot_status[iRobot] == 0) {
				gp << ", '-' with line lw " << marker_sz << " lc rgb '#1b4f72' notitle";
			} else {
				gp << ", '-' with line lw " << marker_sz << " lc rgb '#196f3d' notitle";
			}
		}
		for(size_t iRobot = 0; iRobot < num_robots_; ++iRobot) {
			if(robot_status[iRobot] == 0) {
				gp << ",'-' with points pt 7 ps " << marker_sz << " lc rgb '#1b4f72' notitle";
			} else {
				gp << ",'-' with points pt 7 ps " << marker_sz << " lc rgb '#196f3d' notitle";
				/* gp << ",'-' with points pt 7 ps 2 lc rgb '#196f3d' notitle"; */
			}
		}
		gp << "\n";
		for(long int i = 0; i < system_map_.rows(); ++i) {
			for(long int j = 0; j < system_map_.cols(); ++j) {
				gp << system_map_(i, j) << " ";
			}
			gp << "\n";
		}
		gp << "e\n";
		for(size_t iRobot = 0; iRobot < num_robots_; ++iRobot) {
			for(auto const &pos : robot_positions_history_[iRobot]) {
				gp << pos[0] << " " << pos[1] << std::endl;
			}
			gp << "e\n";
		}

		for(size_t iRobot = 0; iRobot < num_robots_; ++iRobot) {
			gp << robot_global_positions_[iRobot][0] << " " << robot_global_positions_[iRobot][1] << std::endl;
			gp << "e\n";
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

		Gnuplot gp;
		std::string marker_sz;
		GnuplotCommands(gp, params_.pWorldMapSize * params_.pResolution, marker_sz);

		gp << "set o '" << map_filename << ".png'\n";
		std::string res = std::to_string(params_.pResolution);
		gp << "plot '-' matrix using ($2*" << res << "):($1*" << res << "):3 with image notitle ";
		gp << ",'-' with points pt 7 ps " << marker_sz << " lc rgb '#1b4f72' notitle";
		gp << "\n";
		auto world_map = GetWorldIDF();
		for(long int i = 0; i < world_map.rows(); ++i) {
			for(long int j = 0; j < world_map.cols(); ++j) {
				gp << world_map(i, j) << " ";
			}
			gp << "\n";
		}
		gp << "e\n";
		for(size_t iRobot = 0; iRobot < num_robots_; ++iRobot) {
			gp << robot_global_positions_[iRobot][0] << " " << robot_global_positions_[iRobot][1] << std::endl;
		}
		gp << "e\n";
	}

	void CoverageSystem::PlotFrontiers(std::string const &dir_name, int const &step, PointVector const &frontiers) const {
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

		Gnuplot gp;
		std::string marker_sz;
		GnuplotCommands(gp, params_.pWorldMapSize * params_.pResolution, marker_sz);

		gp << "set o '" << map_filename << ".png'\n";

		std::string res = std::to_string(params_.pResolution);
		gp << "plot '-' matrix using ($2*" << res << "):($1*" << res << "):3 with image notitle ";
		// Plot trajectory trails
		gp << ", '-' with line lw " << marker_sz << " lc rgb '#1b4f72' notitle";
		for(size_t iRobot = 0; iRobot < num_robots_; ++iRobot) {
			gp << ",'-' with points pt 7 ps " << marker_sz << " lc rgb '#1b4f72' notitle";
		}
			gp << ",'-' with points pt 1 ps " << marker_sz << " lc rgb 'red' notitle";
		gp << "\n";
		for(long int i = 0; i < system_map_.rows(); ++i) {
			for(long int j = 0; j < system_map_.cols(); ++j) {
				gp << system_map_(i, j) << " ";
			}
			gp << "\n";
		}
		gp << "e\n";
		for(size_t iRobot = 0; iRobot < num_robots_; ++iRobot) {
			for(auto const &pos : robot_positions_history_[iRobot]) {
				gp << pos[0] << " " << pos[1] << std::endl;
			}
			if(iRobot < num_robots_ - 1) {
				gp << "\n";;
			} else {
				gp << "e\n";;
			}
		}

		for(size_t iRobot = 0; iRobot < num_robots_; ++iRobot) {
			gp << robot_global_positions_[iRobot][0] << " " << robot_global_positions_[iRobot][1] << std::endl;
			gp << "e\n";;
		}
		for(auto const &f:frontiers) {
			gp << f[0] << " " << f[1] << std::endl;
		}
		gp << "e\n";;

	}
}	// namespace CoverageControl
