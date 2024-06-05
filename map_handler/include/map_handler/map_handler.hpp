#ifndef MAP_HANLDER__MAP_HANDLER_HPP
#define MAP_HANDLER__MAP_HANDLER_HPP

#include <openvdb/openvdb.h>
#include <map_handler/fov.hpp>

namespace map_handler{
	template <class T>
	class MapHandler{
			std::shared_ptr<openvdb::Grid<T>> grid_;
			std::shared_ptr<fov::FoV> fov_ {nullptr}; // yet to be implemented

			std::string fixed_frame_;
			std::string map_frame_;  // If these two are the same, then no rotation applied to the map
		public:
			MapHandler();

			void insertBox();
			void insertCone();
			void insertSphere();

			void setFixedFrame(const std::string fixed_frame);
			void setMapFrame(const std::string map_frame);

			void rotateMap();  // perform the grid rotation from map frame to fixed frame

			void flushFoV();  // removes everything inside the FoV; to be performed as a TopologyDifference
	};
}  // namespace map_handler
#endif