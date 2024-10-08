#ifndef REPRESENTATION_PLUGINS__PLUGIN_BOX_TEST_HPP
#define REPRESENTATION_PLUGINS__PLUGIN_BOX_TEST_HPP

#include <memory>
#include <chrono>
#include <functional>

#include <rclcpp/rclcpp.hpp>

#include "representation_plugin_base/plugin_base.hpp"
#include <map_handler/semantic_map_handler.hpp>

#include <vdb2pc/vdb2pc_publisher.hpp>

namespace representation_plugins{
	class SemanticPlugin: public PluginBase {
		private:
			rclcpp::TimerBase::SharedPtr timer_;

			std::shared_ptr<map_handler::SemanticMapHandler> semantic_map_;
			std::shared_ptr<vdb2pc::ros_utils::VDB2PCPublisher<openvdb::Int32Grid>> map_publisher_;

			double omega_;
		public:
			PluginBoxTest();
			~PluginBoxTest();
			void run() = 0;
	};
} // namespace representation_plugins
#endif