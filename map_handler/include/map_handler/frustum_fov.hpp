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

				std::shared_ptr<rclcpp::Node> node_;

				rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr debug_viz_pub_;
				visualization_msgs::msg::Marker frustum_msg_;

				void generateDebuggingMessage(const std::string & ref_frame);

			public:
				FrustumFOV(std::shared_ptr<tf2_ros::Buffer> tf_buffer,
						   const double & h_fov, const double & v_fov,
						   const double & d_min, const double & d_max,
						   const tf2::Vector3 & axis);

				template <class T>
				bool inFovPoint(const T & point){
					tf2::Vector3 point_vec(point.x, point.y, point.z);
					tf2::Vector3 comparison_vec;
					for (auto& plane: planes_){
						comparison_vec = point_vec-plane.first;
						comparison_vec.normalize();
						double cos_theta = comparison_vec.dot(plane.second);
						if (cos_theta < 0){
							return false;
						}
					}
					return true;
				}

				void setupDebuggingInterface(std::shared_ptr<rclcpp::Node> node,
											 const std::string & ref_frame);

				inline void publishDebugginMessage(){
					frustum_msg_.header.stamp = node_->get_clock()->now();
					debug_viz_pub_->publish(frustum_msg_);
				}

				bool inFovPoint(const geometry_msgs::msg::PointStamped & point){
					return true;
				}
		};
	}
}
#endif