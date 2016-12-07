#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <functional>
#include <random>
#include <Eigen/Geometry>
#include <visualization_msgs/Marker.h>
#include <uncertainty_planning_core/uncertainty_planning_core.hpp>
#include <uncertainty_planning_examples/config_common.hpp>
#include <uncertainty_planning_examples/se2_common_config.hpp>
#include <uncertainty_planning_examples/se3_common_config.hpp>
#include <uncertainty_planning_examples/baxter_common_config.hpp>
#include <uncertainty_planning_examples/ur5_common_config.hpp>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

using namespace uncertainty_contact_planning;

void demonstrate_se3(ros::Publisher& display_debug_publisher)
{
    std::cout << "Demonstrating SE(3)..." << std::endl;
    const uncertainty_planning_core::PLANNING_AND_EXECUTION_OPTIONS options = se3_common_config::GetOptions();
    const config_common::EXTRA_CONFIG_PARAMS extra_options = se3_common_config::GetExtraOptions();
    std::cout << PrettyPrint::PrettyPrint(options) << PrettyPrint::PrettyPrint(extra_options) << std::endl;
    const std::pair<uncertainty_planning_core::SE3Config, uncertainty_planning_core::SE3Config> start_and_goal = se3_common_config::GetStartAndGoal();
    const uncertainty_planning_core::SE3SamplerPtr sampler = se3_common_config::GetSampler();
    const simple_robot_models::SE3_ROBOT_CONFIG robot_config = se3_common_config::GetDefaultRobotConfig(extra_options);
    const uncertainty_planning_core::SE3Robot robot = se3_common_config::GetRobot(robot_config);
    const uncertainty_planning_core::SE3SimulatorPtr simulator = se3_common_config::GetSimulator(extra_options, options.debug_level);
    uncertainty_planning_core::DemonstrateSE3Simulator(options, robot, simulator, sampler, start_and_goal.first, start_and_goal.second, display_debug_publisher);
}

void demonstrate_se2(ros::Publisher& display_debug_publisher)
{
    std::cout << "Demonstrating SE(2)..." << std::endl;
    const uncertainty_planning_core::PLANNING_AND_EXECUTION_OPTIONS options = se2_common_config::GetOptions();
    const config_common::EXTRA_CONFIG_PARAMS extra_options = se2_common_config::GetExtraOptions();
    std::cout << PrettyPrint::PrettyPrint(options) << PrettyPrint::PrettyPrint(extra_options) << std::endl;
    const std::pair<uncertainty_planning_core::SE2Config, uncertainty_planning_core::SE2Config> start_and_goal = se2_common_config::GetStartAndGoal();
    const uncertainty_planning_core::SE2SamplerPtr sampler = se2_common_config::GetSampler();
    const simple_robot_models::SE2_ROBOT_CONFIG robot_config = se2_common_config::GetDefaultRobotConfig(extra_options);
    const uncertainty_planning_core::SE2Robot robot = se2_common_config::GetRobot(robot_config);
    const uncertainty_planning_core::SE2SimulatorPtr simulator = se2_common_config::GetSimulator(extra_options, options.debug_level);
    uncertainty_planning_core::DemonstrateSE2Simulator(options, robot, simulator, sampler, start_and_goal.first, start_and_goal.second, display_debug_publisher);
}

void demonstrate_baxter(ros::Publisher& display_debug_publisher)
{
    std::cout << "Demonstrating Baxter..." << std::endl;
    const uncertainty_planning_core::PLANNING_AND_EXECUTION_OPTIONS options = baxter_linked_common_config::GetOptions();
    const config_common::EXTRA_CONFIG_PARAMS extra_options = baxter_linked_common_config::GetExtraOptions();
    std::cout << PrettyPrint::PrettyPrint(options) << PrettyPrint::PrettyPrint(extra_options) << std::endl;
    const std::vector<double> joint_uncertainty_params = baxter_linked_common_config::GetJointUncertaintyParams(extra_options);
    assert(joint_uncertainty_params.size() == 7);
    const std::vector<double> joint_distance_weights = baxter_linked_common_config::GetJointDistanceWeights();
    assert(joint_distance_weights.size() == 7);
    const std::pair<uncertainty_planning_core::LinkedConfig, uncertainty_planning_core::LinkedConfig> start_and_goal = baxter_linked_common_config::GetStartAndGoal();
    const uncertainty_planning_core::LinkedSamplerPtr sampler = baxter_linked_common_config::GetSampler();
    const simple_robot_models::LINKED_ROBOT_CONFIG robot_config = baxter_linked_common_config::GetDefaultRobotConfig(extra_options);
    const Eigen::Affine3d base_transform = baxter_linked_common_config::GetBaseTransform();
    const uncertainty_planning_core::LinkedRobot robot = baxter_linked_common_config::GetRobot(base_transform, robot_config, joint_uncertainty_params, joint_distance_weights, extra_options.environment_id);
    const uncertainty_planning_core::LinkedSimulatorPtr simulator = baxter_linked_common_config::GetSimulator(extra_options, options.debug_level);
    uncertainty_planning_core::DemonstrateLinkedSimulator(options, robot, simulator, sampler, start_and_goal.first, start_and_goal.second, display_debug_publisher);
}

void demonstrate_ur5(ros::Publisher& display_debug_publisher)
{
    std::cout << "Demonstrating UR5..." << std::endl;
    const uncertainty_planning_core::PLANNING_AND_EXECUTION_OPTIONS options = ur5_linked_common_config::GetOptions();
    const config_common::EXTRA_CONFIG_PARAMS extra_options = ur5_linked_common_config::GetExtraOptions();
    std::cout << PrettyPrint::PrettyPrint(options) << PrettyPrint::PrettyPrint(extra_options) << std::endl;
    const std::vector<double> joint_uncertainty_params = ur5_linked_common_config::GetJointUncertaintyParams(extra_options);
    assert(joint_uncertainty_params.size() == 6);
    const std::vector<double> joint_distance_weights = ur5_linked_common_config::GetJointDistanceWeights();
    assert(joint_distance_weights.size() == 6);
    const std::pair<uncertainty_planning_core::LinkedConfig, uncertainty_planning_core::LinkedConfig> start_and_goal = ur5_linked_common_config::GetStartAndGoal();
    const uncertainty_planning_core::LinkedSamplerPtr sampler = ur5_linked_common_config::GetSampler();
    const simple_robot_models::LINKED_ROBOT_CONFIG robot_config = ur5_linked_common_config::GetDefaultRobotConfig(extra_options);
    const Eigen::Affine3d base_transform = ur5_linked_common_config::GetBaseTransform();
    const uncertainty_planning_core::LinkedRobot robot = ur5_linked_common_config::GetRobot(base_transform, robot_config, joint_uncertainty_params, joint_distance_weights);
    const uncertainty_planning_core::LinkedSimulatorPtr simulator = ur5_linked_common_config::GetSimulator(extra_options, options.debug_level);
    uncertainty_planning_core::DemonstrateLinkedSimulator(options, robot, simulator, sampler, start_and_goal.first, start_and_goal.second, display_debug_publisher);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simulator_demonstration_node");
    ros::NodeHandle nh;
    ROS_INFO("Starting Simulator Demonstration Node...");
    ros::Publisher display_debug_publisher = nh.advertise<visualization_msgs::MarkerArray>("uncertainty_planning_debug_display_markers", 1, true);
    std::string robot_type;
    ros::NodeHandle nhp("~");
    nhp.param(std::string("robot_type"), robot_type, std::string("baxter"));
    if (robot_type == "se2")
    {
        demonstrate_se2(display_debug_publisher);
    }
    else if (robot_type == "se3")
    {
        demonstrate_se3(display_debug_publisher);
    }
    else if (robot_type == "baxter")
    {
        demonstrate_baxter(display_debug_publisher);
    }
    else if (robot_type == "ur5")
    {
        demonstrate_ur5(display_debug_publisher);
    }
    else
    {
        std::cout << "Robot type [" << robot_type << "] is not recognized" << std::endl;
    }
    return 0;
}
