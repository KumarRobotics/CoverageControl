/**
 *
 **/

#ifndef COVERAGECONTROL_LLOYD_ALGORITHMS_H_
#define COVERAGECONTROL_LLOYD_ALGORITHMS_H_

#include <vector>
#include <fstream>
#include <iostream>
#include <random>
#include <omp.h>
#include <time.h>
#define EIGEN_NO_CUDA // Don't use eigen's cuda facility
#include <Eigen/Dense> // Eigen is used for maps

#include "parameters.h"
#include "typedefs.h"
#include <lsap/Hungarian.h>

namespace CoverageControl {

	inline auto LloydOffline(int const num_tries, int const max_iterations, int const num_sites, MapType const &map, int const map_size, double const res) {
		std::random_device rd_;  //Will be used to obtain a seed for the random number engine
		std::mt19937 gen_;
		std::srand(time(NULL));
		gen_ = std::mt19937(rd_()); //Standard mersenne_twister_engine seeded with rd_()
		std::vector <Voronoi> all_voronoi_cells;
		/* all_voronoi_cells.resize(num_tries, std::vector<VoronoiCell>(num_sites)); */
		all_voronoi_cells.resize(num_tries);
		std::vector <double> obj_values;
		obj_values.resize(num_tries, 0);
		std::uniform_real_distribution<> distrib_pts(0, map_size * res);

#pragma omp parallel for
		for(int iter = 0; iter < num_tries; ++iter) {
			PointVector sites;
			sites.resize(num_sites);
			for(int iSite = 0; iSite < num_sites; ++iSite) {
				sites[iSite] = Point2(distrib_pts(gen_), distrib_pts(gen_));
			}
			bool cont_flag = true;
			/* std::cout << "voronoi start" << std::endl; */
			Voronoi voronoi(sites, map, map_size, res);
			/* std::cout << "voronoi end" << std::endl; */
			auto voronoi_cells = voronoi.GetVoronoiCells();
			int iSteps = 0;
			for(iSteps = 0; iSteps < max_iterations and cont_flag == true; ++iSteps) {
				cont_flag = false;
				voronoi_cells = voronoi.GetVoronoiCells();
				for(int iSite = 0; iSite < num_sites; ++iSite) {
					Point2 diff = voronoi_cells[iSite].centroid - voronoi_cells[iSite].site;
					if(diff.norm() < res) {
						continue;
					}
					cont_flag = true;
					sites[iSite] = voronoi_cells[iSite].centroid;
				}
				voronoi.UpdateSites(sites);
			}
			/* std::cout << "No. of voronoi steps: " << iSteps << std::endl; */
			all_voronoi_cells[iter] = voronoi;
			obj_values[iter] = voronoi.GetObjValue();
		}
		int best_vornoi_idx = 0;
		double min = obj_values[0];
		for(int iter = 1; iter < num_tries; ++iter) {
			if(obj_values[iter] < min) {
				min = obj_values[iter];
				best_vornoi_idx = iter;
			}
		}
		return all_voronoi_cells[best_vornoi_idx];
	}

	inline auto LloydOffline(int const num_tries, int const max_iterations, int const num_sites, MapType const &map, int const map_size, double const res, PointVector const &positions) {

		auto voronoi = LloydOffline(num_tries, max_iterations, num_sites, map, map_size, res);
		auto voronoi_cells = voronoi.GetVoronoiCells();
		std::vector <std::vector<double>> cost_matrix;
		cost_matrix.resize(num_sites, std::vector<double>(num_sites));
#pragma omp parallel for num_threads(num_sites)
		for(int iRobot = 0; iRobot < num_sites; ++iRobot) {
			for(int jCentroid = 0; jCentroid < num_sites; ++jCentroid) {
				cost_matrix[iRobot][jCentroid] = (positions[iRobot] - voronoi_cells[jCentroid].centroid).norm();
			}
		}
		HungarianAlgorithm HungAlgo;
		std::vector<int> assignment;
		HungAlgo.Solve(cost_matrix, assignment);

		PointVector goals;
		goals.resize(num_sites);
		for(int iRobot = 0; iRobot < num_sites; ++iRobot) {
			goals[iRobot] = voronoi_cells[assignment[iRobot]].centroid;
		}

		return goals;
	}

} /* namespace CoverageControl */
#endif /* COVERAGECONTROL_LLOYD_ALGORITHMS_H_ */
