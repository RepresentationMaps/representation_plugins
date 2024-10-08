#include "representation_plugins/semantic_plugin.hpp"

namespace representation_plugins{
	SemanticPlugin::SemanticPlugin():
		PluginBase(),
		simulation_(false){}

	SemanticPlugin::SemanticPlugin(
		std::shared_ptr<map_handler::SemanticMapHandler> & semantic_map,
		std::shared_ptr<representation_plugins::RegionsRegister> & regions_register):
		PluginBase(regions_register)
		{
			semantic_map_ = semantic_map;
		}

	SemanticPlugin::~SemanticPlugin(){
		timer_.reset();
		regions_register_.reset();
		semantic_map_.reset();
	}

	void SemanticPlugin::initializeSimulationStructures(){
		start_simulation_subscription_ = plugin_node_ptr_->create_subscription<std_msgs::msg::Bool>(
				"start_simulation", 10, 
				std::bind(&SemanticPlugin::startSimulationCallback, this, std::placeholders::_1));
			stop_simulation_subscription_ = plugin_node_ptr_->create_subscription<std_msgs::msg::Bool>(
				"stop_simulation", 10, 
				std::bind(&SemanticPlugin::stopSimulationCallback, this, std::placeholders::_1));
			step_subscription_ = plugin_node_ptr_->create_subscription<std_msgs::msg::Float32>(
				"step_size", 10,
				std::bind(&SemanticPlugin::stepCallback, this, std::placeholders::_1));
			forward_subscription_ = plugin_node_ptr_->create_subscription<std_msgs::msg::Bool>(
				"forward_step", 10,
				std::bind(&SemanticPlugin::forwardCallback, this, std::placeholders::_1));
			backward_subscription_ = plugin_node_ptr_->create_subscription<std_msgs::msg::Bool>(
				"backward_step", 10,
				std::bind(&SemanticPlugin::backwardCallback, this, std::placeholders::_1));
	}

	void SemanticPlugin::startSimulationCallback(const std_msgs::msg::Bool::SharedPtr msg){
		if (simulation_){
			return;
		}
		simulation_ = true;
		simulated_time_ = plugin_node_ptr_->get_clock()->now().seconds();
	}

	void SemanticPlugin::stopSimulationCallback(const std_msgs::msg::Bool::SharedPtr msg){
		simulation_ = false;
	}

	void SemanticPlugin::stepCallback(const std_msgs::msg::Float32::SharedPtr msg){
		step_size_ = msg->data;
	}

	void SemanticPlugin::forwardCallback(const std_msgs::msg::Bool::SharedPtr msg){
		simulated_time_ += step_size_;
	}

	void SemanticPlugin::backwardCallback(const std_msgs::msg::Bool::SharedPtr msg){
		simulated_time_ -= step_size_;
	}
}