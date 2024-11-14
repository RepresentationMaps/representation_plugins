Representation Plugins
======================

Here you can find three different packages:

1. `map_handler`, where we provide a VDB-wrapper for semantic data handling.
2. `representation_plugin_base`, where we provide the base class for the Representation Plugins. It also includes the definition of the class implementing the lookup table for the unions of instances as described in the paper.
3. `representation_plugins`, where we provide the implementation of some plugins, handling spatially-grounded semantic information. In this case, you can find two example plugins (`PluginBoxTest`, `PluginAnotherBoxTest`) where we display the usage of the various tools part of this project.

Other relevant reMap repos
--------------------------

In order to run the `reMap` demo part of the HRI 2025 submission, other `reMap` packages are required:

- [`representation_manager`](https://anonymous.4open.science/r/representation_manager/README.md)
- [`reg_of_space_server`](https://anonymous.4open.science/r/reg_of_space_server/README.md)
- [`vdb2pc`](https://anonymous.4open.science/r/vdb2pc/README.md)
- [`simulation_player`](https://anonymous.4open.science/r/simulation_player/README.md)

Maintenance Note
-----------------

### Maintenance and Future Development

The authors of this repository are committed to maintaining and actively developing this code for the years to come. This work is part of the broader `reMap` project. The code will be maintained within the `reMap` GitHub organization to ensure a cohesive and well-supported ecosystem across all related repositories.

### Project Support

The development of the entire `reMap` project is financially supported by **Anonymized Company Name**. This support enables us to dedicate resources for continuous improvements, bug fixes, and the addition of new features. While the `reMap` framework is designed to be generic and adaptable across different platforms, **Anonymized Company Name** is committed to integrating reMap into its robotic products, thereby actively promoting and advancing the project.

### Open-Source Commitment

We are committed to keeping this repository, along with the entire `reMap` project, available under a permissive open-source license. This ensures that the code will remain free to use, modify, and distribute, fostering an open and collaborative environment for the community.

We welcome contributions from users and developers. If you have any suggestions or encounter any issues, please feel free to open an issue or submit a pull request.