#include <iomanip>
#include <iostream>
#include <random>
#include <string>
#include <cstdlib>

#include <GeographicLib/LocalCartesian.hpp>
#include <CoverageControl/geographiclib_wrapper.h>

int main(int argc, char** argv) {
	CoverageControl::GeoLocalTransform geo_transform(40.74050005471615, -74.1759877275644, 0);
	auto lla = geo_transform.Reverse(100, 100, 0);
	std::cout << "lat: " << lla[0] << std::endl;
	std::cout << "lon: " << lla[1] << std::endl;
	std::cout << "hgt: " << lla[2] << std::endl;

	auto xyz = geo_transform.Forward(lla[0], lla[1], lla[2]);
	std::cout << "x: " << xyz[0] << std::endl;
	std::cout << "y: " << xyz[1] << std::endl;
	std::cout << "z: " << xyz[2] << std::endl;

	GeographicLib::LocalCartesian geo_transform1(40.74050005471615, -74.1759877275644);
	geo_transform1.Forward(lla[0], lla[1], lla[2], xyz[0], xyz[1], xyz[2]);
	std::cout << "x: " << xyz[0] << std::endl;
	std::cout << "y: " << xyz[1] << std::endl;
	std::cout << "z: " << xyz[2] << std::endl;

	return 0;
}
