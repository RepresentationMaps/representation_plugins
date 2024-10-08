#include <representation_plugins/plugin_base.hpp>

namespace representation_plugins{
	PluginBase::PluginBase(){}

	PluginBase::PluginBase(std::shared_ptr<RegionsRegister> & regions_register){
		regions_register_ = regions_register;
		jsdnjdnf
	}

	void PluginBase::assignNodeInterface(std::shared_ptr<rclcpp::Node> node_ptr){
		node_ptr_ = node_ptr;
	}
}