/*!
 * This file is part of the CoverageControl library
 * Class to plot the map
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

#ifndef COVERAGECONTROL_PLOTTER_H_
#define COVERAGECONTROL_PLOTTER_H_

#include <iostream>
#include <list>
#include <vector>
#include <filesystem>
#include "typedefs.h"
#include "voronoi.h"

class Gnuplot;
namespace CoverageControl {

	//! Data structure to store plotter data
	struct PlotterData {
		MapType map;
		PointVector positions;
		std::vector <std::list<Point2>> positions_history;
		std::vector <int> robot_status;
		std::vector <std::list<Point2>> voronoi;
	};

	//! Class to plot the map
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

		std::string color_robot = "#002d7d";
		std::string color_robot_alt = "#196f3d";
		/* std::string color_idf = "#900C3F"; */
		std::string color_idf = "#900C3F";
		std::string color_voronoi = "#196f3d";
		std::string color_unknown = "#aeb6bf";
		std::string color_communication_links = "#1f77b4";

		public:
		Plotter(std::string const &d, int const &r_max, double const &res) {
			SetDir(d);
			range_max = r_max;
			resolution = res;
		}

		inline void SetDir(std::string const &d) { dir = d; }

		inline void SetScale(double const &sc) {
			scale = sc;
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

		/* void PlotMap(MapType const &); */
		/* void PlotMap(MapType const &, PointVector const &); */
		/* void PlotMap(MapType const &, PointVector const &); */
		/* void PlotMap(MapType const &, PointVector const &, std::vector <std::list<Point2>> const &, std::vector <int> const &); */
		/* void PlotMap(MapType const &, PointVector const &, PointVector const &, Voronoi const &); */
		/* void PlotMap(MapType const &, PointVector const &, PointVector const &); */

		void GnuplotCommands(Gnuplot &gp);
		void StreamMap(Gnuplot &gp, MapType const &map);
		void PlotMap(Gnuplot &gp, bool begin = true);
		void PlotLine(Gnuplot &gp, int marker_sz, std::string color, bool begin = false);
		void PlotPoints(Gnuplot &gp, int point_type, int marker_sz, std::string color, bool begin = false);

		void PlotMap(MapType const &map);

		void PlotMap(MapType const &map, PointVector const &positions);

		void PlotMap(MapType const &map, PointVector const &positions, std::vector <std::list<Point2>> const &trajectories, std::vector<int> const &robot_status);
		void PlotMap(MapType const &map, PointVector const &positions, std::vector <std::list<Point2>> const &trajectories, std::vector<int> const &robot_status, double const &communication_range);

		void PlotMap(MapType const &map, PointVector const &positions, std::vector<std::list<Point2>> const &voronoi, std::vector <std::list<Point2>> const &trajectories);
		void PlotMap(MapType const &map, PointVector const &positions, Voronoi const &voronoi, std::vector <std::list<Point2>> const &trajectories);
		void PlotMap(MapType const &map, PointVector const &positions, PointVector const &goals, Voronoi const &voronoi);

		void PlotMap(MapType const &map, PointVector const &positions, std::vector <std::list<Point2>> const &trajectories, PointVector const &frontiers);

	};


}

#endif /* COVERAGECONTROL_PLOTTER_H_ */
