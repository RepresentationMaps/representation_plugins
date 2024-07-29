#ifndef REPRESENTATION_PLUGINS__PLUGIN_BASE_HPP
#define REPRESENTATION_PLUGINS__PLUGIN_BASE_HPP

#include <memory>

#include <rclcpp/rclcpp.hpp>

namespace representation_plugins{
	class PluginBase{
		private:
			std::shared_ptr<rclcpp::Node> node_ptr_;
		public:
			PluginBase();
			void assignNodeInterface(std::shared_ptr<rclcpp::Node> node_ptr);
	};
}  // representation_plugins
#endif