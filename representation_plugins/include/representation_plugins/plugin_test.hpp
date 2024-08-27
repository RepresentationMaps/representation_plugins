#ifndef REPRESENTATION_PLUGINS__PLUGIN_TEST_HPP
#define REPRESENTATION_PLUGINS__PLUGIN_TEST_HPP

#include <memory>
#include <chrono>
#include <functional>

#include <rclcpp/rclcpp.hpp>

#include <representation_plugin_base/plugin_base.hpp>

namespace representation_plugins{
	class PluginTest: public PluginBase {
		private:
			rclcpp::TimerBase::SharedPtr timer_;
		public:
			PluginTest();
			~PluginTest();
			void run() const;
			void initialize() override;
	};
} // namespace representation_plugins
#endif