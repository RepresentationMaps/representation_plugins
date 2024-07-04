#ifndef MAP_HANLDER__FRUSTUM_FOV_HPP
#define MAP_HANDLER__FRUSTUM_FOV_HPP

#include "map_handler/fov.hpp"

#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <rclcpp/rclcpp.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <vector>

namespace map_handler{
	namespace fov{
		class FrustumFOV: public FOV {
				// Frustum geometrical parameters
				double h_fov_;
				double v_fov_;
				double d_min_;
				double d_max_;

				tf2::Vector3 axis_; // This is expected to be already expressed in the reference frame
				tf2::Quaternion quaternion_; // The quaternion used to rotate the fake axis over the real one

				std::map<tf2::Vector3, tf2::Vector3> planes_; // a map representing the frustum planes
															  // first = plane point, second = plane normal
				std::vector<tf2::Vector3> vertices_; // a, b, c, d, e, f, g, h

				std::shared_ptr<rclcpp::Node> node_; // node interface to create the marker publisher

				rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr debug_viz_pub_; // debugging publisher - frustum
				rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr debug_point_viz_pub_; // debugging publisher - frustum
				visualization_msgs::msg::Marker frustum_msg_; // debugging marker message - frustum
				visualization_msgs::msg::Marker point_to_check_; // debugging marker message - point to check
				rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr set_point_sub_;

				void generateDebuggingMessage(const std::string & ref_frame);

				void pointToTestCb(const geometry_msgs::msg::PointStamped::SharedPtr msg);

			public:
				FrustumFOV(std::shared_ptr<tf2_ros::Buffer> tf_buffer,
						   const double & h_fov, const double & v_fov,
						   const double & d_min, const double & d_max,
						   const tf2::Vector3 & axis);

				bool inFovPoint(const double & x, const double & y, const double & z);

				void setupDebuggingInterface(std::shared_ptr<rclcpp::Node> node,
											 const std::string & ref_frame);

				inline void publishDebugginMessage(){
					frustum_msg_.header.stamp = node_->get_clock()->now();
					debug_viz_pub_->publish(frustum_msg_);
					debug_point_viz_pub_->publish(point_to_check_);
				}
		};
	}
}
#endif