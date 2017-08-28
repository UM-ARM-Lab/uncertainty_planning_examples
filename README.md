# uncertainty_planning_examples

This package provides examples of using our framework for motion planning and execution with actuation uncertainty. More information on the planning and execution methods can be found in our WAFR 2016 [paper](http://arm.eecs.umich.edu/download.php?p=54) and [presentation](https://www.youtube.com/watch?v=42rwqAUTlbo&list=PL24TB_XE22Jvx6Ozhmdwl5kRClbWjUS0m).

### This package provides several examples of using the planner and execution policy system:

- SE(2)

- SE(3)

- Baxter

## Dependencies

- [arc_utilities](https://github.com/UM-ARM-LAB/arc_utilities)
 
Provides a range of utility and math functions.

- [sdf_tools](https://github.com/UM-ARM-LAB/sdf_tools)

Tools for modeling environments using voxel grids, including several types of collision maps, signed distance fields, and optional integration with MoveIt!

- [uncertainty_planning_core](https://github.com/UM-ARM-LAB/uncertainty_planning_core)

Provides the planner itself and the execution policy system.

- [fast_kinematic_simulator](https://github.com/UM-ARM-LAB/fast_kinematic_simulator)

Provides the fast kinematic simulator used while planning and tools for building simulation environments.

- [ROS Kinetic](http://ros.org)

ROS is required for the build system, Catkin, and for RViz, which the simulator uses as an optional visualization interface.

## Examples

See the `src` directory for examples of using the simulator, planner, and execution policy. 
