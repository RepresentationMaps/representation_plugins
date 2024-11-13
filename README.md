Building reMap
==============

1. clone and build `OpenVDB`. Due to broken dependencies, we can not rely
	on the packaged version of `OpenVDB` available through `apt` for
	Ubuntu 22. To build `OpenVDB` from source, following the
	instructions from [the official github repo](https://github.com/AcademySoftwareFoundation/openvdb)
	should be enough. **Important**: build version `11.0.0`.
2. Clone the following repos:
	1. [`reg_of_space_server`](https://gitlab/lorenzoferrini/reg_of_space_server), checkout `devel` branch.
	2. [`representation_manager`](https://gitlab/lorenzoferrini/representation_manager), checkout `devel` branch.
	3. [`representation_plugins`](https://gitlab/lorenzoferrini/representation_plugins), checkout `regions-of-space` branch.
	4. [`vdb2pc`](https://gitlab/lorenzoferrini/vdb2pc), checkout `devel` branch.
3. Initialize `rosdep`, update and run this command to install all the dependencies:
	`rosdep install -y --skip-keys map_handler --skip-keys representation_plugin_base --skip-keys vdb2pc --skip-keys OpenCV --skip-keys openvdb --skip-keys reg_of_space_server --skip-keys representation_plugins --from-path src
`.
4. Time to build! `colcon build --packages-select vdb2pc reg_of_space_server representation_plugin_base map_handler representation_plugins representation_manager simulation_player --cmake-args "-DFIND_OPENVDB_PATH=/usr/local/lib/cmake/OpenVDB"`

Demo time!
----------

1. `ros2 run reg_of_space_server reg_of_space_server_node`;
2. `ros2 run representation_manager representation_manager_node`
3. `ros2 service call /add_plugin representation_manager/srv/AddPlugin "plugin_name: 'PluginBoxTest'"`
4. Start `rviz2`, set the fixed frame to `map` and visualize the pointcloud published
	on the topic `/PluginBoxTest/test_map`. That's it!