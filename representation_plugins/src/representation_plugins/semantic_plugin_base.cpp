#include "representation_plugins/semantic_plugin.hpp"

namespace representation_plugins{
	SemanticPlugin::SemanticPlugin():
		PluginBase(){
			period_ms_ = 50;  // By default timer @ 20 Hz
		}

	SemanticPlugin::~SemanticPlugin(){
		timer_.reset();
	}
}