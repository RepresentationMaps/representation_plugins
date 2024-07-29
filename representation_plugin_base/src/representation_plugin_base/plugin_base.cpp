#include <representation_plugin_base/plugin_base.hpp>

namespace representation_plugins{
	PluginBase::PluginBase(){}

	void PluginBase::assignNodeInterface(std::shared_ptr<rclcpp::Node> node_ptr){
		node_ptr_ = node_ptr;
	}
}