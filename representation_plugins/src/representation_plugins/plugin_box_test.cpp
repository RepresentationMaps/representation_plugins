#include <cmath>

#include "representation_plugins/plugin_box_test.hpp"

namespace representation_plugins{
	PluginBoxTest::PluginBoxTest():
		PluginBase(){}

	PluginBoxTest::~PluginBoxTest(){
		timer_.reset();
		semantic_map_.reset();
		map_.reset();
	}

	void PluginBoxTest::initialize() {
		RCLCPP_INFO(node_ptr_->get_logger(), "PluginBoxTest is initializing");
		timer_ = node_ptr_->create_wall_timer(
			std::chrono::milliseconds(50),
			std::bind(&PluginBoxTest::run, this));
		semantic_map_ = std::make_shared<map_handler::MapHandler<openvdb::Int32Grid>>(
			threaded_,
			0.1,
			true);
		map_publisher_ = std::make_shared<vdb2pc::ros_utils::VDB2PCPublisher<openvdb::Int32Grid>>(
			plugin_node_ptr_,
			"/PluginBoxTest/test_map",
			"test_frame");
		omega_ = 0.7;
	}

	void PluginBoxTest::run() {
		auto request = std::make_shared<reg_of_space_server::srv::RegOfSpace::Request>();
		request->plugin_name = name_;
		request->n_regs = 2;
		std::vector<std::string> regs_id;
		auto result = register_client_->async_send_request(request);
		if (rclcpp::spin_until_future_complete(plugin_node_ptr_, result) == 
			rclcpp::FutureReturnCode::SUCCESS)
		{
			regs_id = result.get()->regs_of_space_id;
			auto reg_of_space_id_1 = regs_id[0];
			auto reg_of_space_id_2 = regs_id[1];
			auto start = std::chrono::high_resolution_clock::now();
	      	semantic_map_->insertSemanticBox(
	      		1.0,
	      		1.0,
	      		1.0,
	      		reg_of_space_id_1,
	      		*regions_register_,
	      		openvdb::Vec3d(3*std::sin(omega_*(plugin_node_ptr_->get_clock()->now().seconds())), 0.0, 0.0));
	      	auto end = std::chrono::high_resolution_clock::now();
	      	std::chrono::duration<double> elapsed = end - start;
	      	std::cout << "Elapsed time: " << elapsed.count() << " seconds" << std::endl;
	      	semantic_map_->insertSemanticBox(
	      		1.0,
	      		1.0,
	      		1.0,
	      		reg_of_space_id_2,
	      		*regions_register_,
	      		openvdb::Vec3d(-3*std::sin(omega_*(plugin_node_ptr_->get_clock()->now().seconds())), 0.0, 0.0));
	      	map_publisher_->publish(*(semantic_map_->getGridPtr()));
		} else {
			RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
		}

		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Number of regions: %d", regions_register_->getRegionsNumber());

		auto remove_request = std::make_shared<reg_of_space_server::srv::RemoveRegOfSpace::Request>();
		remove_request->regs_of_space_id = regs_id;
		auto remove_result = remove_client_->async_send_request(remove_request);

		if (rclcpp::spin_until_future_complete(plugin_node_ptr_, remove_result) == 
			rclcpp::FutureReturnCode::SUCCESS)
		{
			if (remove_result.get()->result){
				// RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Regions succesfully removed from server");
			}
		} else {
			RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
		}

		regions_register_->print();
		semantic_map_->clear();
		regions_register_->clear();
	}
}  // representation_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(representation_plugins::PluginBoxTest, representation_plugins::PluginBase)