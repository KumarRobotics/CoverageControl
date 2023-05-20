#include "../include/CoverageControl/extern/gnuplot/gnuplot-iostream.h"
#include "../include/CoverageControl/plotter.h"

namespace CoverageControl {

	void Plotter::GnuplotCommands(Gnuplot &gp) {
		std::filesystem::path path{dir};
		if(not std::filesystem::exists(path)) {
			std::cerr << "Directory does not exist" << std::endl;
			throw std::runtime_error{"Directory does not exist"};
		}
		std::string map_filename {std::filesystem::absolute(path/plot_name)};
		gp << "set o '" << map_filename << "'\n";
		gp << "set terminal pngcairo enhanced font 'Times," << font_sz << "' size " << image_sz << "," << image_sz <<"\n";
		gp << "set palette defined (-5 'black', -1 '" << color_unknown << "', 0 'white', 1 '" << color_idf << "')\n";
		gp << "set cbrange [-5:1]\n";
		gp << "set size ratio -1\n";
		gp << "set xrange [0:" << range_max << "]\n";
		gp << "set yrange [0:" << range_max << "]\n";
		if(unset_colorbox)
			gp << "unset colorbox\n";
	}

	void Plotter::StreamMap(Gnuplot &gp, MapType const &map) {
		for(long int i = 0; i < map.rows(); ++i) {
			for(long int j = 0; j < map.cols(); ++j) {
				gp << map(i, j) << " ";
			}
			gp << "\n";
		}
		gp << "e\n";
	}

	void Plotter::PlotMap(Gnuplot &gp, bool begin) {
		if(begin == true)
			gp << "plot ";
		else
			gp << ", ";
		gp << "'-' matrix using ($2*" << resolution << "):($1*" << resolution << "):3 with image notitle ";
	}

	void Plotter::PlotLine(Gnuplot &gp, int marker_sz, std::string color, bool begin) {
		if(begin == true)
			gp << "plot ";
		else
			gp << ", ";
		gp << "'-' with line lw " << marker_sz << " lc rgb '" << color << "' notitle";
	}

	void Plotter::PlotPoints(Gnuplot &gp, int point_type, int marker_sz, std::string color, bool begin) {
		if(begin == true)
			gp << "plot ";
		else
			gp << ", ";
		gp << "'-' with points pt " << point_type << " ps " << marker_sz << " lc rgb '" << color << "' notitle";
	}

	void Plotter::PlotMap(MapType const &map) {
		Gnuplot gp;
		std::string marker_sz;
		GnuplotCommands(gp);
		PlotMap(gp);
		gp << "\n";
		StreamMap(gp, map);
	}

	void Plotter::PlotMap(MapType const &map, PointVector const &positions) {
		Gnuplot gp;
		GnuplotCommands(gp);
		PlotMap(gp);
		PlotPoints(gp, 7, marker_sz, color_robot);
		gp << "\n";

		StreamMap(gp, map);

		for(auto const &pos : positions) {
			gp << pos[0] << " " << pos[1] << std::endl;
		}
		gp << "e\n";

	}

	void Plotter::PlotMap(MapType const &map, PointVector const &positions, std::vector <std::list<Point2>> const &trajectories, std::vector<int> const &robot_status) {

		Gnuplot gp;
		GnuplotCommands(gp);
		PlotMap(gp);

		for(size_t i = 0; i < positions.size(); ++i) {
			if(robot_status[i] == 0) {
				PlotLine(gp, marker_sz, color_robot, false);
			} else {
				PlotLine(gp, marker_sz, color_robot_alt, false);
			}
		}
		for(size_t i = 0; i < positions.size(); ++i) {
			if(robot_status[i] == 0) {
				PlotPoints(gp, 7, marker_sz, color_robot, false);
			} else {
				PlotPoints(gp, 7, marker_sz, color_robot_alt, false);
			}
		}
		gp << "\n";

		StreamMap(gp, map);
		for(auto const &trajectory : trajectories) {
			for(auto const &pos : trajectory) {
				gp << pos[0] << " " << pos[1] << std::endl;
			}
			gp << "e\n";
		}

		for(auto const &pos : positions) {
			gp << pos[0] << " " << pos[1] << std::endl;
			gp << "e\n";
		}
	}

	void Plotter::PlotMap(MapType const &map, PointVector const &positions, PointVector const &goals, Voronoi const &voronoi) {

		Gnuplot gp;
		GnuplotCommands(gp);
		PlotMap(gp);

		PlotLine(gp, half_marker_sz, color_voronoi, false); // voronoi
		PlotLine(gp, half_marker_sz, color_robot, false); // goals path
		PlotPoints(gp, 28, marker_sz, color_robot, false); // goals
		PlotPoints(gp, 7, marker_sz, color_robot, false); // robots
		gp << "\n";

		StreamMap(gp, map);

		auto voronoi_cells = voronoi.GetVoronoiCells();
		for(auto const &vcell : voronoi_cells) {
			for(auto const &pos : vcell.cell) {
				gp << pos[0] << " " << pos[1] << std::endl;
			}
			auto const &pos = vcell.cell.front();
			gp << pos[0] << " " << pos[1] << std::endl;
			gp << "\n";
		}
		gp << "e\n";

		for(size_t i = 0; i < positions.size(); ++i) {
			auto const &pos = positions[i];
			auto const &goal = goals[i];
			gp << pos[0] << " " << pos[1] << std::endl;
			gp << goal[0] << " " << goal[1] << std::endl;
			gp << "\n";
		}
		gp << "e\n";

		for(auto const &pos : goals) {
			gp << pos[0] << " " << pos[1] << std::endl;
		}
		gp << "e\n";

		for(auto const &pos : positions) {
			gp << pos[0] << " " << pos[1] << std::endl;
		}
		gp << "e\n";
	}

	void Plotter::PlotMap(MapType const &map, PointVector const &positions, std::vector <std::list<Point2>> const &trajectories, PointVector const &frontiers) {

		Gnuplot gp;
		GnuplotCommands(gp);

		PlotMap(gp);
		PlotLine(gp, marker_sz, color_robot);
		PlotPoints(gp, 7, marker_sz, color_robot);
		PlotPoints(gp, 1, half_marker_sz, color_robot);

		gp << "\n";

		StreamMap(gp, map);
		for(auto const &trajectory : trajectories) {
			for(auto const &pos : trajectory) {
				gp << pos[0] << " " << pos[1] << std::endl;
			}
			gp << "\n";
		}
		gp << "e\n";

		for(auto const &pos : positions) {
			gp << pos[0] << " " << pos[1] << std::endl;
		}
		gp << "e\n";

		for(auto const &pos : frontiers) {
			gp << pos[0] << " " << pos[1] << std::endl;
		}
		gp << "e\n";

	}



}
