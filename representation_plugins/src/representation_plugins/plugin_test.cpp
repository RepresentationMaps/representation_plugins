#include "representation_plugins/plugin_test.hpp"

namespace representation_plugins{
	PluginTest::PluginTest(){
		id_ = 1;
	}
}  // representation_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(representation_plugins::PluginTest, representation_plugins::PluginBase)