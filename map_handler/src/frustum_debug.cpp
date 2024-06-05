#include <memory>
#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <tf2/LinearMath/Vector3.h>

#include  "map_handler/frustum_fov.hpp"

int main(int argc, char ** argv){
	rclcpp::init(argc, argv);

	std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("frustum_debug");

	auto tf_buffer =
      std::make_shared<tf2_ros::Buffer>(node->get_clock());
    auto tf_listener =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  double h_fov = 0.2;
  double v_fov = 0.2;
  double d_min = 0.5;
  double d_max = 5.0;

  tf2::Vector3 axis(0.0, 0.0, 1.0);

	map_handler::fov::FrustumFOV frustum(tf_buffer,
					   h_fov, v_fov, d_min, d_max,
					   axis);

  frustum.setupDebuggingInterface(node, "world");

  rclcpp::Rate loop_rate(5);

  while(rclcpp::ok()){
    frustum.publishDebugginMessage();
    loop_rate.sleep();
  }

  rclcpp::shutdown();
}