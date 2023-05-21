#include <iomanip>
#include <iostream>
#include <random>
#include <string>
#include <cstdlib>
#include <fstream>
#include <chrono>

#include <CoverageControl/constants.h>
#include <CoverageControl/parameters.h>
#include <CoverageControl/typedefs.h>
#include <CoverageControl/bivariate_normal_distribution.h>
#include <CoverageControl/world_idf.h>
#include <CoverageControl/robot_model.h>
#include <CoverageControl/generate_world_map.ch>
#include <CoverageControl/coverage_system.h>

using namespace CoverageControl;

int main(int argc, char** argv) {
	Parameters params("/home/saurav/CoverageControl_ws/src/CoverageControl/core/params/parameters.yaml");
	int num_robots = 20;
	CoverageSystem env(params, 100, num_robots);
	/* std::cout << "Env created" << std::endl; */
	std::string map_filename = "data/oracle_map";
	std::string gnuplot_script = "src/CoverageControl/scripts/gnuplot/plot_map.gp";

	bool plot_map = true;
	bool write_data = false;
	auto start = std::chrono::steady_clock::now();
	for(int iNum = 0; iNum < 1; ++iNum) {
		std::string data_filename = "data/cnn_data.ssv";
		std::ofstream out_file(data_filename);
		if(!out_file) {
			throw std::runtime_error {"Cannot open file"};
		}
		for(int i = 0; i < 100; ++i) {
			std::cout << iNum << " " << i << std::endl;
			env.StepDataGenerationLocal(10);
			if(write_data) {
				for(int iRobot = 0; iRobot < num_robots; ++iRobot) {
					auto robot_local_map = env.GetRobotLocalMap(iRobot);
					auto comm_map = env.GetCommunicationMap(iRobot);
					auto voronoi_cell = env.GetVoronoiCell(iRobot);
					Point2 centroid = voronoi_cell.centroid;
					double mass = voronoi_cell.mass;
					for(int ix = 0; ix < params.pLocalMapSize; ++ix) {
						for(int iy = 0; iy < params.pLocalMapSize; ++iy) {
							out_file << robot_local_map(ix, iy) << " ";
						}
					}
					for(int ix = 0; ix < params.pLocalMapSize; ++ix) {
						for(int iy = 0; iy < params.pLocalMapSize; ++iy) {
							out_file << comm_map(ix, iy) << " ";
						}
					}
					out_file << centroid.x() << " " << centroid.y() << " " << mass << std::endl;
				}
			}

			if(plot_map == true) {
				auto oracle_map = env.GetCommunicationMap(0);
				std::stringstream ss;
				ss << std::setw(4) << std::setfill('0') << i;
				std::string s = ss.str();
				std::string imap_name = map_filename + s;
				MapUtils::WriteMap(oracle_map, imap_name + ".dat");
				std::string gnuplot_command = "gnuplot -c " + gnuplot_script + " " + imap_name + " 1 " + std::to_string(params.pResolution) + " " + std::to_string(params.pLocalMapSize * params.pResolution);
				std::system(gnuplot_command.c_str());
			}
		}
		out_file.close();
		if(write_data) {
			std::stringstream ss;
			ss << std::setw(4) << std::setfill('0') << iNum;
			std::string s = ss.str();
			std::string tar_command = "tar -Jcvf ";
			tar_command += "data/cnn_data/cnn_data_" + s +  ".tar.xz data/cnn_data.ssv";
			std::system(tar_command.c_str());
		}

		auto end = std::chrono::steady_clock::now();
		std::chrono::duration<double> elapsed_seconds = end-start;
		std::cout << "elapsed time: " << elapsed_seconds.count() << "s\n";
	}
	return 0;
}
