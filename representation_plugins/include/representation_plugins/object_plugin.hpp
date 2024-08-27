#ifndef REPRESENTATION_PLUGINS__PLUGIN_TEST_HPP
#define REPRESENTATION_PLUGINS__PLUGIN_TEST_HPP

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <representation_plugin_base/plugin_base.hpp>

namespace representation_plugins{
	class ObjectsPlugin: public PluginBase {
		private:
			int id_;
		public:
			ObjectsPlugin(){};
	};
} // namespace representation_plugins
#endif