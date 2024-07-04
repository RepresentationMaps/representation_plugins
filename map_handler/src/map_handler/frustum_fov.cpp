#include <cmath>
#include <utility>
#include <functional>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometry_msgs/msg/point.hpp>

#include <tf2/LinearMath/Scalar.h>

#include "map_handler/frustum_fov.hpp"

namespace map_handler{
	namespace fov{
		FrustumFOV::FrustumFOV(std::shared_ptr<tf2_ros::Buffer> tf_buffer,
							   const double & h_fov, const double & v_fov,
							   const double & d_min, const double & d_max,
							   const tf2::Vector3 & axis):
			FOV(tf_buffer),
			h_fov_(h_fov), v_fov_(v_fov), d_min_(d_min), d_max_(d_max),
			axis_(axis){
				// 1. width @ d_min
				double width_min = d_min*std::tan(h_fov);
				// width_min *= 2;

				// 2. width @ d_max
				double width_max = d_max*std::tan(h_fov);
				// width_max *= 2;

				// 3. height @ d_min
				double height_min = d_min*std::tan(v_fov);
				// height_min *= 2;				

				// 4. height @ d_max
				double height_max = d_max*std::tan(v_fov);
				// height_max *= 2;

				// 5. First we assume that the frustum axis
				// 	  is aligned with the [1, 0, 0] axis.
				//    Then, we procees
				axis_.normalize();
				tf2::Vector3 temp_axis(1.0, 0.0, 0.0);

				// 5.1 We compute the angle between the two vectors
				//	   assumes the angle is in range [0, pi]
				double angle = std::acos(temp_axis.dot(axis_));

				// 5.2 Compute the cross product to find a normal
				//     vector to later be used for the quaternion computation
				tf2::Vector3 rot_axis = temp_axis.cross(axis_);
				rot_axis.normalize(); // shouldn't be required

				// 5.3 We rotate the vector along the axis and we check
				// 	   whether the rotation was succesful
				tf2::Vector3 rotated_axis = temp_axis.rotate(rot_axis, angle);
				tf2::Vector3 diff_axis = rotated_axis-axis_;
				if (diff_axis.length() > 1e-3){
					rot_axis = -rot_axis;
					rotated_axis = temp_axis.rotate(rot_axis, angle);
					diff_axis = rotated_axis-axis_;
					if (diff_axis.length() > 1e-3){
						std::cout<<"Warning: could not compute the correct rotation axis. Setting the rotation axis to [1, 0, 0]"<<std::endl;
						rot_axis.setX(1.0);
						rot_axis.setY(0.0);
						rot_axis.setZ(0.0);

						angle = 0.0;
					}
				}

				// 5.4 compute the quaternion representing the rotation
				quaternion_ = tf2::Quaternion(rot_axis, angle);

				tf2::Vector3 center_min(d_min, 0.0, 0.0);
				tf2::Vector3 center_max(d_max, 0.0, 0.0);

				// 5.5 we compute the temporary coordinates for the frustum volume
				tf2::Vector3 ab, bc, ef, fg; // All of these are actually half of the vector
				ab = tf2::Vector3(0.0, width_min, 0.0);
				bc = tf2::Vector3(0.0, 0.0, height_min);
				ef = tf2::Vector3(0.0, width_max, 0.0);
				fg = tf2::Vector3(0.0, 0.0, height_max);

				// 5.6 we compute the rotated limits
				tf2::Vector3 a, b, c, d, e, f, g, h;
				a = center_min+ab-bc;
				a = tf2::quatRotate(quaternion_, a);
				vertices_.push_back(a);
				b = center_min-ab-bc;
				b = tf2::quatRotate(quaternion_, b);
				vertices_.push_back(b);
				c = center_min-ab+bc;
				c = tf2::quatRotate(quaternion_, c);
				vertices_.push_back(c);
				d = center_min+ab+bc;
				d = tf2::quatRotate(quaternion_, d);
				vertices_.push_back(d);
				e = center_max+ef-fg;
				e = tf2::quatRotate(quaternion_, e);
				vertices_.push_back(e);
				f = center_max-ef-fg;
				f = tf2::quatRotate(quaternion_, f);
				vertices_.push_back(f);
				g = center_max-ef+fg;
				g = tf2::quatRotate(quaternion_, g);
				vertices_.push_back(g);
				h = center_max+ef+fg;
				h = tf2::quatRotate(quaternion_, h);
				vertices_.push_back(h);

				// 5.7 we compute the normal vectors to the six faces
				tf2::Vector3 ba, cb;
				ba = a-b;
				ba.normalize();
				cb = b-c;
				cb.normalize();
				tf2::Vector3 p = cb.cross(ba);
				p.normalize();
				planes_.insert(planes_.begin(), std::make_pair<tf2::Vector3, tf2::Vector3>(std::move(a), std::move(p)));

				tf2::Vector3 bf;
				bf = f-b;
				bc = -cb;
				p = bc.cross(bf);
				p.normalize();
				planes_.insert(planes_.begin(), std::make_pair<tf2::Vector3, tf2::Vector3>(std::move(b), std::move(p)));

				p = bf.cross(ba);
				p.normalize();
				planes_.insert(planes_.begin(), std::make_pair<tf2::Vector3, tf2::Vector3>(std::move(f), std::move(p)));

				tf2::Vector3 ae, ad;
				ae = e-a;
				ad = d-a;
				p = ae.cross(ad);
				p.normalize();
				planes_.insert(planes_.begin(), std::make_pair<tf2::Vector3, tf2::Vector3>(std::move(d), std::move(p)));

				tf2::Vector3 fe;
				fg = g-f;
				fe = e-f;
				p = fg.cross(fe);
				p.normalize();
				planes_.insert(planes_.begin(), std::make_pair<tf2::Vector3, tf2::Vector3>(std::move(e), std::move(p)));

				tf2::Vector3 cd, cg;
				cd = d-c;
				cg = g-c;
				p = cd.cross(cg);
				p.normalize();
				planes_.insert(planes_.begin(), std::make_pair<tf2::Vector3, tf2::Vector3>(std::move(c), std::move(p)));

				// At this point, we completed the limits of the volume and its faces normals, pointing internally
		}

		bool FrustumFOV::inFovPoint(const double & x, const double & y, const double & z){
			tf2::Vector3 point_vec(x, y, z);
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

		void FrustumFOV::generateDebuggingMessage(const std::string & ref_frame){
			frustum_msg_.header.frame_id = ref_frame;
			
			frustum_msg_.type = visualization_msgs::msg::Marker::LINE_LIST;
			
			frustum_msg_.action = visualization_msgs::msg::Marker::ADD;
			
			frustum_msg_.scale.x = 0.1;
			frustum_msg_.scale.y = 0.0;
			frustum_msg_.scale.z = 0.0;

			frustum_msg_.color.r = 1.0;
			frustum_msg_.color.g = 0.0;
			frustum_msg_.color.b = 0.0;
			frustum_msg_.color.a = 1.0;

			frustum_msg_.frame_locked = true;

			// ab
			geometry_msgs::msg::Point p1, p2;

			p1.x = vertices_[0][0]; p1.y = vertices_[0][1]; p1.z = vertices_[0][2];
			p2.x = vertices_[1][0]; p2.y = vertices_[1][1]; p2.z = vertices_[1][2];
			frustum_msg_.points.push_back(p1);
			frustum_msg_.points.push_back(p2);
			// bc
			p1.x = vertices_[1][0]; p1.y = vertices_[1][1]; p1.z = vertices_[1][2];
			p2.x = vertices_[2][0]; p2.y = vertices_[2][1]; p2.z = vertices_[2][2];
			frustum_msg_.points.push_back(p1);
			frustum_msg_.points.push_back(p2);
			// cd
			p1.x = vertices_[2][0]; p1.y = vertices_[2][1]; p1.z = vertices_[2][2];
			p2.x = vertices_[3][0]; p2.y = vertices_[3][1]; p2.z = vertices_[3][2];
			frustum_msg_.points.push_back(p1);
			frustum_msg_.points.push_back(p2);
			// ad
			p1.x = vertices_[0][0]; p1.y = vertices_[0][1]; p1.z = vertices_[0][2];
			p2.x = vertices_[3][0]; p2.y = vertices_[3][1]; p2.z = vertices_[3][2];
			frustum_msg_.points.push_back(p1);
			frustum_msg_.points.push_back(p2);
			// ef
			p1.x = vertices_[4][0]; p1.y = vertices_[4][1]; p1.z = vertices_[4][2];
			p2.x = vertices_[5][0]; p2.y = vertices_[5][1]; p2.z = vertices_[5][2];
			frustum_msg_.points.push_back(p1);
			frustum_msg_.points.push_back(p2);
			// fg
			p1.x = vertices_[5][0]; p1.y = vertices_[5][1]; p1.z = vertices_[5][2];
			p2.x = vertices_[6][0]; p2.y = vertices_[6][1]; p2.z = vertices_[6][2];
			frustum_msg_.points.push_back(p1);
			frustum_msg_.points.push_back(p2);
			// gh
			p1.x = vertices_[6][0]; p1.y = vertices_[6][1]; p1.z = vertices_[6][2];
			p2.x = vertices_[7][0]; p2.y = vertices_[7][1]; p2.z = vertices_[7][2];
			frustum_msg_.points.push_back(p1);
			frustum_msg_.points.push_back(p2);
			// eh
			p1.x = vertices_[4][0]; p1.y = vertices_[4][1]; p1.z = vertices_[4][2];
			p2.x = vertices_[7][0]; p2.y = vertices_[7][1]; p2.z = vertices_[7][2];
			frustum_msg_.points.push_back(p1);
			frustum_msg_.points.push_back(p2);
			// ae
			p1.x = vertices_[4][0]; p1.y = vertices_[4][1]; p1.z = vertices_[4][2];
			p2.x = vertices_[0][0]; p2.y = vertices_[0][1]; p2.z = vertices_[0][2];
			frustum_msg_.points.push_back(p1);
			frustum_msg_.points.push_back(p2);
			// bf
			p1.x = vertices_[1][0]; p1.y = vertices_[1][1]; p1.z = vertices_[1][2];
			p2.x = vertices_[5][0]; p2.y = vertices_[5][1]; p2.z = vertices_[5][2];
			frustum_msg_.points.push_back(p1);
			frustum_msg_.points.push_back(p2);
			// cg
			p1.x = vertices_[2][0]; p1.y = vertices_[2][1]; p1.z = vertices_[2][2];
			p2.x = vertices_[6][0]; p2.y = vertices_[6][1]; p2.z = vertices_[6][2];
			frustum_msg_.points.push_back(p1);
			frustum_msg_.points.push_back(p2);
			// dh
			p1.x = vertices_[3][0]; p1.y = vertices_[3][1]; p1.z = vertices_[3][2];
			p2.x = vertices_[7][0]; p2.y = vertices_[7][1]; p2.z = vertices_[7][2];
			frustum_msg_.points.push_back(p1);
			frustum_msg_.points.push_back(p2);
			
			// Now creating the marker object 
			// for the point to evaluate whether in or out of the frustum
			point_to_check_.header.frame_id = ref_frame;

			point_to_check_.type = visualization_msgs::msg::Marker::SPHERE;
			
			point_to_check_.action = visualization_msgs::msg::Marker::ADD;

			point_to_check_.scale.x = 0.1;
			point_to_check_.scale.y = 0.1;
			point_to_check_.scale.z = 0.1;

			point_to_check_.pose.position.x = 0.0;
			point_to_check_.pose.position.y = 0.0;
			point_to_check_.pose.position.z = 0.0;

			point_to_check_.pose.orientation.x = 0.0;
			point_to_check_.pose.orientation.y = 0.0;
			point_to_check_.pose.orientation.z = 0.0;
			point_to_check_.pose.orientation.w = 1.0;

			point_to_check_.color.r = 1.0;
			point_to_check_.color.g = 0.0;
			point_to_check_.color.b = 0.0;
			point_to_check_.color.a = 1.0;

			point_to_check_.frame_locked = true;
		}

		void FrustumFOV::setupDebuggingInterface(std::shared_ptr<rclcpp::Node> node, const std::string & ref_frame){
			node_ = node;
			debug_viz_pub_ = node->create_publisher<visualization_msgs::msg::Marker>("frustum", 1);
			debug_point_viz_pub_ = node->create_publisher<visualization_msgs::msg::Marker>("point", 1);
			set_point_sub_ = node->create_subscription<geometry_msgs::msg::PointStamped>(
				"point_to_check", 1, std::bind(&FrustumFOV::pointToTestCb, this, std::placeholders::_1));

			// generate the debugging visualization message
			generateDebuggingMessage(ref_frame);
			frustum_msg_.header.stamp = node->get_clock()->now();
		}

		void FrustumFOV::pointToTestCb(const geometry_msgs::msg::PointStamped::SharedPtr msg){
			geometry_msgs::msg::PointStamped transformed_point;

			if (msg->header.frame_id != ref_frame_){
				try {
			    	auto target_tf = tf_buffer_->lookupTransform(
			    		msg->header.frame_id, ref_frame_, rclcpp::Time(0, 0));
			    	tf2::doTransform(*msg, transformed_point, target_tf);
				} catch (tf2::TransformException & ex) {
			  		RCLCPP_ERROR(node_->get_logger(), "could not transform sellion_target point:\n%s", ex.what());
			  		return;
			  	}
			} else {
				transformed_point.point = msg->point;
			}

		  	point_to_check_.pose.position.x = transformed_point.point.x;
		  	point_to_check_.pose.position.y = transformed_point.point.y;
		  	point_to_check_.pose.position.z = transformed_point.point.z;

		  	if (inFovPoint(transformed_point.point.x, transformed_point.point.y, transformed_point.point.z)){
		  		point_to_check_.color.r = 0.0;
		  		point_to_check_.color.g = 1.0;
		  	} else {
		  		point_to_check_.color.r = 1.0;
		  		point_to_check_.color.g = 0.0;
		  	}
		}
	}
}