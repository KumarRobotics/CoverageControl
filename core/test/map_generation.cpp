#include <iomanip>
#include <iostream>
#include <random>

#include <CoverageControl/constants.h>
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
		Point2 mean(distrib(gen), distrib(gen));
		/* Point2 mean(i * 100, i * 100); */
		double var(distrib_var(gen));
		double peak(distrib_var(gen));
		/* dists_file << mean.x() << " " << mean.y() << " " << var << " " << peak << std::endl; */
		BivariateNormalDistribution dist(mean, var, peak);
		world.AddNormalDistribution(dist);
	}
	/* world.GenerateMap(); */
	world.GenerateMapCuda();
	/* world.WriteMap("data/map.dat"); */
	/* Point2 mean(0,2); */
	/* double var(1); */
	/* BivariateNormalDistribution dist(mean, var); */
	/* world.AddNormalDistribution(dist); */

	dists_file.close();

	return 0;
}
