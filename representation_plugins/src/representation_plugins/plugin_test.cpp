#include "representation_plugins/plugin_test.hpp"

namespace representation_plugins{
	PluginTest::PluginTest():
		PluginBase(){}

	PluginTest::~PluginTest(){
		timer_.reset();
	}

	void PluginTest::initialize() {
		RCLCPP_INFO(node_ptr_->get_logger(), "Plugin Test is initializing");
		timer_ = node_ptr_->create_wall_timer(std::chrono::seconds(1), std::bind(&PluginTest::run, this));
	}

	void PluginTest::run() const {
		RCLCPP_INFO(node_ptr_->get_logger(), "Plugin Test is running");
	}
}  // representation_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(representation_plugins::PluginTest, representation_plugins::PluginBase)