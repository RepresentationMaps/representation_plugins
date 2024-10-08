#include <representation_plugin_base/plugin_base.hpp>

namespace representation_plugins{
	PluginBase::PluginBase(){}

	PluginBase::PluginBase(std::shared_ptr<RegionsRegister> & regions_register){
		regions_register_ = regions_register;
	}

	PluginBase::~PluginBase(){
		remove_client_.reset();
		register_client_.reset();
		node_ptr_.reset();
	}

	void PluginBase::setup(
		const std::shared_ptr<rclcpp::Node> & node_ptr,
		const std::string & name,
		const bool & threaded)
	{
		node_ptr_ = node_ptr;
		plugin_node_ptr_ = std::make_shared<rclcpp::Node>(name);
		name_ = name;
		threaded_ = threaded;
		regions_register_ = std::make_shared<RegionsRegister>(threaded_);
		register_client_ = plugin_node_ptr_->create_client<reg_of_space_server::srv::RegOfSpace>("register_region_of_space");
		remove_client_ = plugin_node_ptr_->create_client<reg_of_space_server::srv::RemoveRegOfSpace>("remove_region_of_space");
	}
}