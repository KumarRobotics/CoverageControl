/**
 * Contains utility functins for transforming maps
 **/

#ifndef COVERAGECONTROL_MAP_UTILS_H_
#define COVERAGECONTROL_MAP_UTILS_H_

#include <cmath>
#include <CoverageControl/parameters.h>

namespace CoverageControl {
	namespace MapUtils {
		struct MapBounds {
			int left = 0, right = 0, bottom = 0, top = 0;
			int width = 0, height = 0;
			void SetZero() {
				left = 0, right = 0, bottom = 0, top = 0;
			}
		};

		// Gets the closest point on the grid
		inline void GetClosestGridCoordinate(double const resolution, Point2 const &pt, int &idx, int &idy) {
			idx = std::round(pt.x()/resolution);
			idy = std::round(pt.y()/resolution);
		}

		// Compute necessary map transformations when the point is close to the boundary
		inline void ComputeOffsets(double const resolution, Point2 const &pos, int const submap_size, int const map_size, MapBounds &index, MapBounds &offset) {
			int pos_idx = 0, pos_idy = 0;
			GetClosestGridCoordinate(resolution, pos, pos_idx, pos_idy);
			index.left = pos_idx - submap_size/2;
			index.right = pos_idx + submap_size/2;
			index.bottom = pos_idy - submap_size/2;
			index.top = pos_idy + submap_size/2;

			offset.SetZero();
			if(index.left < 0) { offset.left = -index.left; }
			if(index.bottom < 0) { offset.bottom = -index.bottom; }

			if(index.right > map_size) { offset.right = map_size - index.right; }
			if(index.top > map_size) { offset.top = map_size - index.top; }

			offset.width = index.right + offset.right - (index.left + offset.left);
			offset.height = index.top + offset.top - (index.bottom + offset.bottom);
		}

		template<typename T = MapType>
		inline void GetSubMap(double const resolution, Point2 const &pos, int const submap_size, int const map_size, T const &map, T &submap) {
			MapBounds index, offset;
			ComputeOffsets(resolution, pos, submap_size, map_size, index, offset);
			submap.block(offset.left, offset.bottom, offset.width, offset.height) = map.block(index.left + offset.left, index.bottom + offset.bottom, offset.width, offset.height);
		}

		/* inline void GetSubMap(double const resolution, Point2 const &pos, int const submap_size, int const map_size, MapType const &map, MapType &submap) { */
		/* 	GetSubMap<> (resolution, pos, submap_size, map_size, map, submap); */
		/* } */

		/** Write the world map to a file **/
		inline int WriteMap(MapType const &map, std::string const &file_name) {
			std::ofstream file_obj(file_name);
			if(!file_obj) {
				std::cerr << "[Error] Could not open " << file_name << " for writing." << std::endl;
				return 1;
			}
			file_obj << map;
			file_obj.close();
			return 0;
		}

		inline int IsPointOutsideBoundary(double const resolution, Point2 const &pos, int const sensor_size, int const boundary) {
			if (pos.x() <= -sensor_size * resolution/2.) { return 1; }
			if (pos.y() <= -sensor_size * resolution/2.) { return 1; }
			if (pos.x() >= boundary * resolution + sensor_size * resolution/2.) { return 1; }
			if (pos.y() >= boundary * resolution + sensor_size * resolution/2.) { return 1; }
			return 0;
		}

	} /* namespace MapUtils */
} /* namespace CoverageControl */
#endif /* COVERAGECONTROL_MAP_UTILS_H_ */

