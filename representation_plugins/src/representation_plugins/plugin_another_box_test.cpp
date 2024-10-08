#include <cmath>

#include "representation_plugins/plugin_another_box_test.hpp"

namespace representation_plugins{
	PluginAnotherBoxTest::PluginAnotherBoxTest():
		SemanticPlugin(){}

	PluginAnotherBoxTest::PluginAnotherBoxTest(
		std::shared_ptr<map_handler::SemanticMapHandler> & semantic_map,
		std::shared_ptr<representation_plugins::RegionsRegister> & regions_register):
		SemanticPlugin(semantic_map, regions_register){}

	PluginAnotherBoxTest::~PluginAnotherBoxTest(){
		timer_.reset();
		semantic_map_.reset();
		regions_register_.reset();
	}

	void PluginAnotherBoxTest::initialize() {
		RCLCPP_INFO(node_ptr_->get_logger(), "PluginAnotherBoxTest is initializing");
		semantic_map_ = std::make_shared<map_handler::SemanticMapHandler>(
			threaded_,
			0.1,
			true);
		omega_ = 0.7;

		auto request = std::make_shared<reg_of_space_server::srv::RegOfSpace::Request>();
		request->plugin_name = name_;
		request->n_regs = 2;
		std::vector<std::string> regs_id;
		auto result = register_client_->async_send_request(request);
		if (rclcpp::spin_until_future_complete(plugin_node_ptr_, result) == 
			rclcpp::FutureReturnCode::SUCCESS)
		{
			regs_id = result.get()->regs_of_space_id;
			reg_of_space_id_1_ = regs_id[0];
			reg_of_space_id_2_ = regs_id[1];
		}
	}

	void PluginAnotherBoxTest::run() {
		auto start = std::chrono::high_resolution_clock::now();
		double current_time = (simulation_)?simulated_time_:plugin_node_ptr_->get_clock()->now().seconds();
      	semantic_map_->insertSemanticBox(
      		1.0,
      		1.0,
      		1.0,
      		reg_of_space_id_1_,
      		*regions_register_,
      		openvdb::Vec3d(0.0, 3*std::sin(omega_*(current_time)), 0.0));
      	semantic_map_->insertSemanticBox(
      		1.0,
      		1.0,
      		1.0,
      		reg_of_space_id_2_,
      		*regions_register_,
      		openvdb::Vec3d(0.0, -3*std::sin(omega_*(current_time)), 0.0));
      	// map_publisher_->publish(*(semantic_map_->getGridPtr()));
      	// regions_register_->print();
      	auto end = std::chrono::high_resolution_clock::now();
      	std::chrono::duration<double> elapsed = end - start;
      	// std::cout << "Elapsed time: " << elapsed.count() << " seconds" << std::endl;
		

		auto remove_request = std::make_shared<reg_of_space_server::srv::RemoveRegOfSpace::Request>();
		std::vector<std::string> regs_id = {reg_of_space_id_1_, reg_of_space_id_2_};
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

		// regions_register_->clear();
	}
}  // representation_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(representation_plugins::PluginAnotherBoxTest, representation_plugins::PluginBase)