#ifndef MAP_HANLDER__FOV_HPP
#define MAP_HANDLER__FOV_HPP

#include <memory>

#include <openvdb/openvdb.h>

#include <tf2_ros/buffer.h>

#include <geometry_msgs/msg/point_stamped.hpp>

namespace map_handler{
	namespace fov{
		class FOV{
			protected:
				std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

				std::string ref_frame_;
			public:
				FOV(std::shared_ptr<tf2_ros::Buffer> tf_buffer):
					tf_buffer_(tf_buffer){}

				virtual bool inFovPoint(const double & x, const double & y, const double & z) = 0;

				void setReferenceFrame(const std::string & ref_frame){
					ref_frame_ = ref_frame;
				}
		};
	}
}
#endif