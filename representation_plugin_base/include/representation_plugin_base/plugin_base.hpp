#ifndef REPRESENTATION_PLUGINS__PLUGIN_BASE_HPP
#define REPRESENTATION_PLUGINS__PLUGIN_BASE_HPP

#include <memory>
#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "representation_plugin_base/regions_register.hpp"

#include "reg_of_space_server/srv/reg_of_space.hpp"
#include "reg_of_space_server/srv/remove_reg_of_space.hpp"

namespace representation_plugins{
	class PluginBase{
		protected:
			std::shared_ptr<rclcpp::Node> node_ptr_;
			std::shared_ptr<rclcpp::Node> plugin_node_ptr_;
			std::string name_;
			bool threaded_;

			std::shared_ptr<RegionsRegister> regions_register_;

			rclcpp::Client<reg_of_space_server::srv::RegOfSpace>::SharedPtr register_client_;
			rclcpp::Client<reg_of_space_server::srv::RemoveRegOfSpace>::SharedPtr remove_client_;
		public:
			PluginBase();
			virtual ~PluginBase();
			void setup(
				const std::shared_ptr<rclcpp::Node> & node_ptr,
				const std::string & name,
				const bool & threaded);
			virtual void initialize() = 0;
	};
}  // representation_plugins
#endif