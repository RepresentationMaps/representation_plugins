#ifndef REPRESENTATION_PLUGINS__PLUGIN_ANOTHER_BOX_TEST_HPP
#define REPRESENTATION_PLUGINS__PLUGIN_ANOTHER_BOX_TEST_HPP

#include <memory>
#include <chrono>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "representation_plugin_base/plugin_base.hpp"
#include "representation_plugins/semantic_plugin.hpp"

#include <vdb2pc/vdb2pc_publisher.hpp>

namespace representation_plugins{
	class PluginAnotherBoxTest: public SemanticPlugin {
		private:
			double omega_;

			std::string reg_of_space_id_1_;
			std::string reg_of_space_id_2_;
		public:
			PluginAnotherBoxTest();
			PluginAnotherBoxTest(
				std::shared_ptr<map_handler::SemanticMapHandler> & semantic_map,
				std::shared_ptr<representation_plugins::RegionsRegister> & regions_register);
			~PluginAnotherBoxTest();
			void run() override;
			void initialize() override;
	};
} // namespace representation_plugins
#endif