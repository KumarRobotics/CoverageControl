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
	Parameters params("/home/saurav/CoverageControl_ws/src/CoverageControl/core/params/parameters.yaml");
	std::srand(0);
	std::random_device rd;  //Will be used to obtain a seed for the random number engine
	std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
	std::uniform_real_distribution<> distrib(0, params.pWorldMapSize * params.pResolution);
	std::uniform_real_distribution<> distrib_var(params.pMinSigma, params.pMaxSigma);
	std::uniform_real_distribution<> distrib_peak(params.pMinPeak, params.pMaxPeak);
	std::ofstream dists_file("data/dists.dat");
	WorldIDF world(params);
	double twicepi = 2. * M_PI;
	for(size_t i = 0; i < 100; ++i) {
		Point2 mean(distrib(gen), distrib(gen));
		/* Point2 mean(i * 100, i * 100); */
		double var(distrib_var(gen));
		double scale(distrib_peak(gen) * twicepi * var * var);
		dists_file << mean.x() << " " << mean.y() << " " << var << " " << scale << std::endl;
		BivariateNormalDistribution dist(mean, var, scale);
		world.AddNormalDistribution(dist);
	}
	
	/* PointVector poly{Point2(100,100), Point2(200,100), Point2(200, 250), Point2(100, 250)}; */
	PointVector poly{Point2(100,100), Point2(200,100), Point2(200, 25), Point2(250, 250)};
	PolygonFeature pf{poly, 0.5};
	world.AddUniformDistributionPolygon(pf);

	/* world.GenerateMap(); */
	world.GenerateMapCuda();
	world.WriteWorldMap("data/map.dat");
	dists_file.close();

	std::string gnuplot_script = "src/CoverageControl/core/scripts/gnuplot/plot_map.gp";
	std::string gnuplot_command = "gnuplot -c " + gnuplot_script + " " + "data/map" + " " + std::to_string(params.pNorm) + " " + std::to_string(params.pResolution) + " " + std::to_string(params.pWorldMapSize * params.pResolution);
	std::system(gnuplot_command.c_str());

	return 0;
}
