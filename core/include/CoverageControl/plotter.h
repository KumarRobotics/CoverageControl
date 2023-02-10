#ifndef COVERAGECONTROL_PLOTTER_H_
#define COVERAGECONTROL_PLOTTER_H_

#include <CoverageControl/typedefs.h>
#include <CoverageControl/voronoi.h>
#include <gnuplot/gnuplot-iostream.h>
#include <iostream>
#include <list>
#include <vector>
#include <filesystem>

namespace CoverageControl {

	template <typename MapType_t = MapType>
		class Plotter {
			std::string dir = "data/test/";
			std::string plot_name = "map.png";
			int marker_sz = 2;
			int half_marker_sz = 1;
			int image_sz = 1024;
			int font_sz = 14;
			double scale = 1;
			bool unset_colorbox = true;

			int range_max = 1024;
			double resolution = 1;

			std::string color_robot = "#1b4f72";
			std::string color_robot_alt = "#196f3d";
			std::string color_idf = "#900C3F";
			std::string color_voronoi = "#196f3d";
			std::string color_unknown = "#aeb6bf";

			public:
			Plotter(std::string const &dir, int const &range_max, double const &resolution) {
				this->dir = dir;
				this->range_max = range_max;
				this->resolution = resolution;
			}

			void SetDir(std::string const &dir) {
				this->dir = dir;
			}

			void SetScale(double const &scale) {
				this->scale = scale;
				marker_sz = int(2 * scale);
				half_marker_sz = int(1 * scale);
				image_sz = int(1024 * scale);
				font_sz = int(14 * scale);
			}

			void SetPlotName(std::string const &name) {
				plot_name = name + ".png";
			}

			void SetPlotName(std::string const &name, int const &i) {
				std::stringstream ss;
				ss << std::setw(4) << std::setfill('0') << i;
				plot_name = name + ss.str() + ".png";
			}

			/* void PlotMap(MapType_t const &); */
			/* void PlotMap(MapType_t const &, PointVector const &); */
			/* void PlotMap(MapType_t const &, PointVector const &); */
			/* void PlotMap(MapType_t const &, PointVector const &, std::vector <std::list<Point2>> const &, std::vector <int> const &); */
			/* void PlotMap(MapType_t const &, PointVector const &, PointVector const &, Voronoi const &); */
			/* void PlotMap(MapType_t const &, PointVector const &, PointVector const &); */

			void GnuplotCommands(Gnuplot &gp) {
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

			void StreamMap(Gnuplot &gp, MapType_t const &map) {
				for(long int i = 0; i < map.rows(); ++i) {
					for(long int j = 0; j < map.cols(); ++j) {
						gp << map(i, j) << " ";
					}
					gp << "\n";
				}
				gp << "e\n";
			}

			void PlotMap(Gnuplot &gp, bool begin = true) {
				if(begin == true)
					gp << "plot ";
				else
					gp << ", ";
				gp << "'-' matrix using ($2*" << resolution << "):($1*" << resolution << "):3 with image notitle ";
			}

			void PlotLine(Gnuplot &gp, int marker_sz, std::string color, bool begin = false) {
				if(begin == true)
					gp << "plot ";
				else
					gp << ", ";
				gp << "'-' with line lw " << marker_sz << " lc rgb '" << color << "' notitle";
			}

			void PlotPoints(Gnuplot &gp, int point_type, int marker_sz, std::string color, bool begin = false) {
				if(begin == true)
					gp << "plot ";
				else
					gp << ", ";
				gp << "'-' with points pt " << point_type << " ps " << marker_sz << " lc rgb '" << color << "' notitle";
			}

			void PlotMap(MapType_t const &map) {
				Gnuplot gp;
				std::string marker_sz;
				GnuplotCommands(gp);
				PlotMap(gp);
				gp << "\n";
				StreamMap(gp, map);
			}

			void PlotMap(MapType_t const &map, PointVector const &positions) {
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

			void PlotMap(MapType_t const &map, PointVector const &positions, std::vector <std::list<Point2>> const &trajectories, std::vector<int> const &robot_status) {

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

			void PlotMap(MapType_t const &map, PointVector const &positions, PointVector const &goals, Voronoi const &voronoi) {

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

			void PlotMap(MapType_t const &map, PointVector const &positions, std::vector <std::list<Point2>> const &trajectories, PointVector const &frontiers) {

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

		};


}

#endif /* COVERAGECONTROL_PLOTTER_H_ */
