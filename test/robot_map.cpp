#include <iomanip>
#include <iostream>
#include <random>
#include <string>
#include <cstdlib>

#include <CoverageControl/constants.h>
#include <CoverageControl/parameters.h>
#include <CoverageControl/typedefs.h>
#include <CoverageControl/bivariate_normal_distribution.h>
#include <CoverageControl/world_idf.h>
#include <CoverageControl/robot_model.h>
#include <CoverageControl/generate_world_map.ch>

using namespace CoverageControl;

int main(int argc, char** argv) {
	std::cout << "Coverage Control" << std::endl;
	std::srand(0);
	std::random_device rd;  //Will be used to obtain a seed for the random number engine
	std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
	std::uniform_real_distribution<> distrib(0, pWorldMapSize * pResolution);
	std::uniform_real_distribution<> distrib_var(2,10);
	std::ofstream dists_file("data/dists.dat");
	WorldIDF world;
	for(size_t i = 0; i < 100; ++i) {
		/* Point2 mean(distrib(gen), distrib(gen)); */
		Point2 mean(i * 100, i * 100);
		double var(distrib_var(gen));
		double peak(distrib_var(gen));
		dists_file << mean.x() << " " << mean.y() << " " << var << " " << peak << std::endl;
		BivariateNormalDistribution dist(mean, var, peak);
		world.AddNormalDistribution(dist);
	}
	world.GenerateMapCuda();
	world.WriteWorldMap("data/map.dat");

	dists_file.close();

	std::string gnuplot_script = "CoverageControl/scripts/gnuplot/plot_map.gp";
	std::string gnuplot_command = "gnuplot -c " + gnuplot_script + " " + "data/map" + " " + std::to_string(world.GetMaxValue()) + " " + std::to_string(pResolution) + " " + std::to_string(pWorldMapSize * pResolution);
	std::system(gnuplot_command.c_str());

	RobotModel robot(Point2(0,0), world);
	std::string robot_map_filename = "data/robot_local_map";
	std::string sensor_filename = "data/sensor";

	for(int i = 0; i < 2; ++i) {
		std::cout << i << std::endl;
		auto local_map = robot.GetRobotLocalMap();
		auto sensor_view = robot.GetSensorView();
		std::string istr = std::to_string(i);
		MapUtils::WriteMap(local_map, robot_map_filename + istr + ".dat");
		MapUtils::WriteMap(sensor_view, sensor_filename + istr + ".dat");
		gnuplot_command = "gnuplot -c " + gnuplot_script + " " + robot_map_filename + istr + " " + std::to_string(world.GetMaxValue()) + " " + std::to_string(pResolution) + " " + std::to_string(pLocalMapSize * pResolution);
		std::system(gnuplot_command.c_str());
		gnuplot_command = "gnuplot -c " + gnuplot_script + " " + sensor_filename + istr + " " + std::to_string(world.GetMaxValue()) + " " + std::to_string(pResolution) + " " + std::to_string(pSensorSize * pResolution);
		std::system(gnuplot_command.c_str());
		robot.StepControl(Point2(sin(M_PI/4.), cos(M_PI/4.)), pMaxRobotSpeed);
		auto robot_position = robot.GetGlobalCurrentPosition();
		std::cout << robot_position.x() << " " << robot_position.y() << std::endl;
	}


	return 0;
}
