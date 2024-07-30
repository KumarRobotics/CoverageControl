/*
 * This file is part of the CoverageControl library
 *
 * Author: Saurav Agarwal
 * Contact: sauravag@seas.upenn.edu, agr.saurav1@gmail.com
 * Repository: https://github.com/KumarRobotics/CoverageControl
 *
 * Copyright (c) 2024, Saurav Agarwal
 *
 * The CoverageControl library is free software: you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * The CoverageControl library is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * CoverageControl library. If not, see <https://www.gnu.org/licenses/>.
 */

/*!
 * \file plotter.cpp
 * \brief Helper function to plot maps
 */

#include <filesystem>
#include <iostream>

#include "CoverageControl/extern/gnuplot/gnuplot-iostream.h"
#include "CoverageControl/plotter.h"

namespace CoverageControl {

[[nodiscard]] bool Plotter::GnuplotCommands(Gnuplot &gp) {
  std::filesystem::path map_filename{
      std::filesystem::weakly_canonical(dir + "/" + plot_name)};
  std::filesystem::path dir_path{map_filename.parent_path()};
  if (!std::filesystem::exists(dir_path)) {
    std::cerr << "Directory does not exist: " << dir_path << std::endl;
    return 1;
  }
  gp << "set o '" << map_filename.string() << "'\n";
  gp << "set terminal pngcairo enhanced font 'Times," << font_sz << "' size "
     << image_sz << "," << image_sz << "\n";
  gp << "set palette defined (-5 'black', -1 '" << color_unknown
     << "', 0 'white', 1 '" << color_idf << "')\n";
  gp << "set cbrange [-5:1]\n";
  gp << "set size ratio -1\n";
  gp << "set xrange [0:" << range_max << "]\n";
  gp << "set yrange [0:" << range_max << "]\n";
  gp << "set border linewidth 1.5\n";
  if (unset_colorbox) gp << "unset colorbox\n";
  return 0;
}

void Plotter::StreamMap(Gnuplot &gp, MapType const &map) {
  for (int i = 0; i < map.rows(); ++i) {
    for (int j = 0; j < map.cols(); ++j) {
      gp << map(i, j) << " ";
    }
    gp << "\n";
  }
  gp << "e" << std::endl;
}

void Plotter::PlotMap(Gnuplot &gp, bool begin) {
  if (begin == true)
    gp << "plot ";
  else
    gp << ", ";
  gp << "'-' matrix using ($2*" << resolution << "):($1*" << resolution
     << "):3 with image notitle ";
}

void Plotter::PlotLine(Gnuplot &gp, int marker_size, std::string color,
                       bool begin) {
  if (begin == true)
    gp << "plot ";
  else
    gp << ", ";
  gp << "'-' with line lw " << marker_size << " lc rgb '" << color
     << "' notitle";
}

void Plotter::PlotPoints(Gnuplot &gp, int point_type, int marker_size,
                         std::string color, bool begin) {
  if (begin == true)
    gp << "plot ";
  else
    gp << ", ";
  gp << "'-' with points pt " << point_type << " ps " << marker_size
     << " lc rgb '" << color << "' notitle";
}

void Plotter::PlotMap(MapType const &map) {
  Gnuplot gp;
  if (GnuplotCommands(gp)) {
    std::cerr << "Error in GnuplotCommands" << std::endl;
    return;
  }
  PlotMap(gp);
  gp << "\n";
  StreamMap(gp, map);
}

void Plotter::PlotMap(MapType const &map, PointVector const &positions) {
  Gnuplot gp;
  if (GnuplotCommands(gp)) {
    std::cerr << "Error in GnuplotCommands" << std::endl;
    return;
  }
  PlotMap(gp);
  PlotPoints(gp, 7, marker_sz, color_robot);
  gp << "\n";

  StreamMap(gp, map);

  for (auto const &pos : positions) {
    gp << pos[0] << " " << pos[1] << std::endl;
  }
  gp << "e" << std::endl;
}

void Plotter::PlotMap(MapType const &map, PointVector const &positions,
                      std::vector<std::list<Point2>> const &trajectories,
                      std::vector<int> const &robot_status) {
  Gnuplot gp;
  if (GnuplotCommands(gp)) {
    std::cerr << "Error in GnuplotCommands" << std::endl;
    return;
  }
  PlotMap(gp);

  for (size_t i = 0; i < positions.size(); ++i) {
    if (robot_status[i] == 0) {
      PlotLine(gp, marker_sz, color_robot, false);
    } else {
      PlotLine(gp, marker_sz, color_robot_alt, false);
    }
  }
  for (size_t i = 0; i < positions.size(); ++i) {
    if (robot_status[i] == 0) {
      PlotPoints(gp, 7, marker_sz, color_robot, false);
    } else {
      PlotPoints(gp, 7, marker_sz, color_robot_alt, false);
    }
  }
  gp << "\n";

  StreamMap(gp, map);
  for (auto const &trajectory : trajectories) {
    for (auto const &pos : trajectory) {
      gp << pos[0] << " " << pos[1] << std::endl;
    }
    gp << "e" << std::endl;
  }

  for (auto const &pos : positions) {
    gp << pos[0] << " " << pos[1] << std::endl;
    gp << "e" << std::endl;
  }
}

void Plotter::PlotMap(MapType const &map, PointVector const &positions,
                      std::vector<std::list<Point2>> const &trajectories,
                      std::vector<int> const &robot_status,
                      double const &communication_range) {
  Gnuplot gp;
  if (GnuplotCommands(gp)) {
    std::cerr << "Error in GnuplotCommands" << std::endl;
    return;
  }
  PlotMap(gp);

  for (size_t i = 0; i < positions.size(); ++i) {
    if (robot_status[i] == 0) {
      PlotLine(gp, marker_sz, color_robot, false);
    } else {
      PlotLine(gp, marker_sz, color_robot_alt, false);
    }
  }
  PlotLine(gp, half_marker_sz, color_communication_links, false);
  for (size_t i = 0; i < positions.size(); ++i) {
    if (robot_status[i] == 0) {
      PlotPoints(gp, 7, marker_sz, color_robot, false);
    } else {
      PlotPoints(gp, 7, marker_sz, color_robot_alt, false);
    }
  }
  gp << "\n";

  StreamMap(gp, map);
  for (auto const &trajectory : trajectories) {
    for (auto const &pos : trajectory) {
      gp << pos[0] << " " << pos[1] << std::endl;
    }
    gp << "e" << std::endl;
  }

  for (size_t i = 0; i < positions.size(); ++i) {
    for (size_t j = i + 1; j < positions.size(); ++j) {
      if ((positions[i] - positions[j]).norm() < communication_range) {
        gp << positions[i][0] << " " << positions[i][1] << "\n";
        gp << positions[j][0] << " " << positions[j][1] << "\n";
        gp << "\n";
      }
    }
  }
  gp << "e" << std::endl;
  for (auto const &pos : positions) {
    gp << pos[0] << " " << pos[1] << std::endl;
    gp << "e" << std::endl;
  }
}

void Plotter::PlotMap(MapType const &map, PointVector const &positions,
                      std::vector<std::list<Point2>> const &voronoi,
                      std::vector<std::list<Point2>> const &trajectories) {
  Gnuplot gp;
  if (GnuplotCommands(gp)) {
    std::cerr << "Error in GnuplotCommands" << std::endl;
    return;
  }
  PlotMap(gp);

  PlotLine(gp, marker_sz, color_robot, false);
  PlotLine(gp, half_marker_sz, color_voronoi, false);  // voronoi
  PlotPoints(gp, 7, marker_sz, color_robot, false);    // robots
  gp << "\n";

  StreamMap(gp, map);

  for (auto const &trajectory : trajectories) {
    for (auto const &pos : trajectory) {
      gp << pos[0] << " " << pos[1] << "\n";
    }
    gp << "\n";
  }
  gp << "e" << std::endl;

  for (auto const &vcell : voronoi) {
    for (auto const &pos : vcell) {
      gp << pos[0] << " " << pos[1] << "\n";
    }
    gp << "\n";
  }
  gp << "e" << std::endl;

  for (auto const &pos : positions) {
    gp << pos[0] << " " << pos[1] << "\n";
  }
  gp << "e" << std::endl;
}

void Plotter::PlotMap(MapType const &map, PointVector const &positions,
                      Voronoi const &voronoi,
                      std::vector<std::list<Point2>> const &trajectories) {
  Gnuplot gp;
  if (GnuplotCommands(gp)) {
    std::cerr << "Error in GnuplotCommands" << std::endl;
    return;
  }
  PlotMap(gp);

  PlotLine(gp, marker_sz, color_robot, false);
  PlotLine(gp, half_marker_sz, color_voronoi, false);  // voronoi
  PlotPoints(gp, 7, marker_sz, color_robot, false);    // robots
  gp << "\n";

  StreamMap(gp, map);

  for (auto const &trajectory : trajectories) {
    for (auto const &pos : trajectory) {
      gp << pos[0] << " " << pos[1] << "\n";
    }
    gp << "\n";
  }
  gp << "e" << std::endl;

  auto voronoi_cells = voronoi.GetVoronoiCells();
  for (auto const &vcell : voronoi_cells) {
    for (auto const &pos : vcell.cell) {
      gp << pos[0] << " " << pos[1] << "\n";
    }
    auto const &pos = vcell.cell.front();
    gp << pos[0] << " " << pos[1] << "\n";
    gp << "\n";
  }
  gp << "e" << std::endl;

  for (auto const &pos : positions) {
    gp << pos[0] << " " << pos[1] << "\n";
  }
  gp << "e" << std::endl;
}

void Plotter::PlotMap(MapType const &map, PointVector const &positions,
                      PointVector const &goals, Voronoi const &voronoi) {
  Gnuplot gp;
  if (GnuplotCommands(gp)) {
    std::cerr << "Error in GnuplotCommands" << std::endl;
    return;
  }
  PlotMap(gp);

  PlotLine(gp, half_marker_sz, color_voronoi, false);  // voronoi
  PlotLine(gp, half_marker_sz, color_robot, false);    // goals path
  PlotPoints(gp, 28, marker_sz, color_robot, false);   // goals
  PlotPoints(gp, 7, marker_sz, color_robot, false);    // robots
  gp << "\n";

  StreamMap(gp, map);

  auto voronoi_cells = voronoi.GetVoronoiCells();
  for (auto const &vcell : voronoi_cells) {
    for (auto const &pos : vcell.cell) {
      gp << pos[0] << " " << pos[1] << std::endl;
    }
    auto const &pos = vcell.cell.front();
    gp << pos[0] << " " << pos[1] << std::endl;
    gp << "\n";
  }
  gp << "e" << std::endl;

  for (size_t i = 0; i < positions.size(); ++i) {
    auto const &pos = positions[i];
    auto const &goal = goals[i];
    gp << pos[0] << " " << pos[1] << std::endl;
    gp << goal[0] << " " << goal[1] << std::endl;
    gp << "\n";
  }
  gp << "e" << std::endl;

  for (auto const &pos : goals) {
    gp << pos[0] << " " << pos[1] << std::endl;
  }
  gp << "e" << std::endl;

  for (auto const &pos : positions) {
    gp << pos[0] << " " << pos[1] << std::endl;
  }
  gp << "e" << std::endl;
}

void Plotter::PlotMap(MapType const &map, PointVector const &positions,
                      std::vector<std::list<Point2>> const &trajectories,
                      PointVector const &frontiers) {
  Gnuplot gp;
  if (GnuplotCommands(gp)) {
    std::cerr << "Error in GnuplotCommands" << std::endl;
    return;
  }

  PlotMap(gp);
  PlotLine(gp, marker_sz, color_robot);
  PlotPoints(gp, 7, marker_sz, color_robot);
  PlotPoints(gp, 1, half_marker_sz, color_robot);

  gp << "\n";

  StreamMap(gp, map);
  for (auto const &trajectory : trajectories) {
    for (auto const &pos : trajectory) {
      gp << pos[0] << " " << pos[1] << std::endl;
    }
    gp << "\n";
  }
  gp << "e" << std::endl;

  for (auto const &pos : positions) {
    gp << pos[0] << " " << pos[1] << std::endl;
  }
  gp << "e" << std::endl;

  for (auto const &pos : frontiers) {
    gp << pos[0] << " " << pos[1] << std::endl;
  }
  gp << "e" << std::endl;
}

}  // namespace CoverageControl
