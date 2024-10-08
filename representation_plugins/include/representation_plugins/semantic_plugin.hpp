#ifndef REPRESENTATION_PLUGINS__SEMANTIC_PLUGIN_HPP
#define REPRESENTATION_PLUGINS__SEMANTIC_PLUGIN_HPP

#include <memory>
#include <chrono>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

#include "representation_plugin_base/plugin_base.hpp"
#include <map_handler/semantic_map_handler.hpp>

#include <vdb2pc/vdb2pc_publisher.hpp>

namespace representation_plugins{
	class SemanticPlugin: public PluginBase {
		protected:
			rclcpp::TimerBase::SharedPtr timer_;
			
			bool simulation_;
			double simulated_time_;
			double step_size_;

			std::shared_ptr<map_handler::SemanticMapHandler> semantic_map_;
			std::shared_ptr<vdb2pc::ros_utils::VDB2PCPublisher<openvdb::Int32Grid>> map_publisher_;

			rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_simulation_subscription_;
			rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_simulation_subscription_;
			rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr step_subscription_;
			rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr forward_subscription_;
			rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr backward_subscription_;
		public:
			SemanticPlugin();
			SemanticPlugin(
				std::shared_ptr<map_handler::SemanticMapHandler> & semantic_map,
				std::shared_ptr<representation_plugins::RegionsRegister> & regions_register);
			virtual ~SemanticPlugin();
			virtual void run() = 0;

			inline void setSemanticMapHandler(
				std::shared_ptr<map_handler::SemanticMapHandler> & semantic_map)
			{
				semantic_map_=semantic_map;
			}
			inline void setRegionsRegister(
				std::shared_ptr<representation_plugins::RegionsRegister> & regions_register)
			{
				regions_register_=regions_register;
			}

			void initializeSimulationStructures();

			void startSimulationCallback(const std_msgs::msg::Bool::SharedPtr msg);
			void stopSimulationCallback(const std_msgs::msg::Bool::SharedPtr msg);
			void stepCallback(const std_msgs::msg::Float32::SharedPtr msg);
			void forwardCallback(const std_msgs::msg::Bool::SharedPtr msg);
			void backwardCallback(const std_msgs::msg::Bool::SharedPtr msg);
	};
} // namespace representation_plugins
#endif