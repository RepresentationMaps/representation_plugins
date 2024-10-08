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
	typedef std::function<void(
		openvdb::Int32Tree&, representation_plugins::RegionsRegister & reg_register)> Filler;
	class IrresponsiblePlugin: public PluginBase {
		protected:
			rclcpp::TimerBase::SharedPtr timer_;

			std::vector<Filler> fillers_;
		public:
			IrresponsiblePlugin();
			virtual ~IrresponsiblePlugin();
			virtual void run() = 0;
			inline std::vector<Filler> getFillers() const {return fillers_};
	};
} // namespace representation_plugins
#endif