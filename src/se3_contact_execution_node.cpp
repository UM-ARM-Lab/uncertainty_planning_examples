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
#include <arc_utilities/eigen_helpers.hpp>
#include <arc_utilities/eigen_helpers_conversions.hpp>
#include <arc_utilities/pretty_print.hpp>
#include <arc_utilities/voxel_grid.hpp>
#include <arc_utilities/simple_rrt_planner.hpp>
#include <uncertainty_planning_core/simple_pid_controller.hpp>
#include <uncertainty_planning_core/simple_uncertainty_models.hpp>
#include <uncertainty_planning_core/uncertainty_contact_planning.hpp>
#include <uncertainty_planning_examples/SetSimple6dofActuationError.h>
#include <uncertainty_planning_examples/se3_common_config.hpp>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <uncertainty_planning_examples/Simple6dofRobotMove.h>

using namespace uncertainty_contact_planning;

inline EigenHelpers::VectorAffine3d move_robot(const Eigen::Affine3d& target_transform, const Eigen::Affine3d& expected_result_configuration, const double duration, const double execution_shortcut_distance, const bool reset, ros::ServiceClient& robot_control_service)
{
    UNUSED(expected_result_configuration);
    const geometry_msgs::PoseStamped target_pose = EigenHelpersConversions::EigenAffine3dToGeometryPoseStamped(target_transform, "world");
    // Put together service call
    uncertainty_planning_examples::Simple6dofRobotMove::Request req;
    req.time_limit = ros::Duration(duration);
    req.execution_shortcut_distance = execution_shortcut_distance;
    if (reset)
    {
        std::cout << "Resetting robot to transform: " << PrettyPrint::PrettyPrint(target_transform) << std::endl;
        req.start = target_pose;
        req.mode = uncertainty_planning_examples::Simple6dofRobotMove::Request::RESET;
    }
    else
    {
        std::cout << "Commanding robot to transform: " << PrettyPrint::PrettyPrint(target_transform) << std::endl;
        req.target = target_pose;
        req.mode = uncertainty_planning_examples::Simple6dofRobotMove::Request::EXECUTE;
    }
    uncertainty_planning_examples::Simple6dofRobotMove::Response res;
    // Call service
    try
    {
        robot_control_service.call(req, res);
    }
    catch (...)
    {
        ROS_ERROR("Move service failed");
    }
    // Unpack result
    const std::vector<geometry_msgs::PoseStamped>& poses = res.trajectory;
    EigenHelpers::VectorAffine3d transforms(poses.size());
    for (size_t idx = 0; idx < poses.size(); idx++)
    {
        transforms[idx] = EigenHelpersConversions::GeometryPoseToEigenAffine3d(poses[idx].pose);
    }
    std::cout << "Reached transform: " << PrettyPrint::PrettyPrint(transforms.back()) << " after " << transforms.size() << " steps" << std::endl;
    return transforms;
}

void set_uncertainty(const double max_translation_error, const double max_rotation_error, ros::ServiceClient& set_uncertainty_service)
{
    uncertainty_planning_examples::SetSimple6dofActuationErrorRequest req;
    req.translation_actuator_error = max_translation_error;
    req.rotation_actuation_error = max_rotation_error;
    uncertainty_planning_examples::SetSimple6dofActuationErrorResponse res;
    if (set_uncertainty_service.call(req, res) == false)
    {
        throw std::invalid_argument("SetActuationError failed");
    }
}

void peg_in_hole_env_se3(ros::Publisher& display_debug_publisher, ros::ServiceClient& robot_control_service, ros::ServiceClient& set_uncertainty_service)
{
    const uncertainty_planning_core::PLANNING_AND_EXECUTION_OPTIONS options = se3_common_config::GetOptions();
    const config_common::TASK_CONFIG_PARAMS extra_options = se3_common_config::GetExtraOptions();
    std::cout << PrettyPrint::PrettyPrint(options) << "\n" << PrettyPrint::PrettyPrint(extra_options) << std::endl;
    const std::pair<uncertainty_planning_core::SE3Config, uncertainty_planning_core::SE3Config> start_and_goal = se3_common_config::GetStartAndGoal();
    const uncertainty_planning_core::SE3SamplerPtr sampler = se3_common_config::GetSampler();
    const simple_robot_models::SE3_ROBOT_CONFIG robot_config = se3_common_config::GetDefaultRobotConfig(extra_options);
    const uncertainty_planning_core::SE3Robot robot = se3_common_config::GetRobot(robot_config);
    const uncertainty_planning_core::SE3SimulatorPtr simulator = se3_common_config::GetSimulator(extra_options, options.debug_level);
    // Load the policy
    try
    {
        auto policy = uncertainty_planning_core::LoadSE3Policy(options.planned_policy_file);
        policy.SetPolicyActionAttemptCount(options.policy_action_attempt_count);
        std::map<std::string, double> complete_policy_stats;
    #ifndef FORCE_DEBUG
        std::cout << "Press ENTER to simulate policy..." << std::endl;
        std::cin.get();
    #endif
        const auto policy_simulation_results = uncertainty_planning_core::SimulateSE3UncertaintyPolicy(options, robot, simulator, sampler, policy, start_and_goal.first, start_and_goal.second, display_debug_publisher);
        const std::map<std::string, double> policy_simulation_stats = policy_simulation_results.second.first;
        const std::vector<int64_t> policy_simulation_step_counts = policy_simulation_results.second.second.first;
        std::cout << "Policy simulation success: " << PrettyPrint::PrettyPrint(policy_simulation_stats) << std::endl;
        complete_policy_stats.insert(policy_simulation_stats.begin(), policy_simulation_stats.end());
    #ifndef FORCE_DEBUG
        std::cout << "Press ENTER to execute policy..." << std::endl;
        std::cin.get();
    #endif
        if (options.num_policy_executions > 0)
        {
            std::cout << "Setting actuation uncertainty..." << std::endl;
            set_uncertainty(robot_config.max_actuator_noise, robot_config.r_max_actuator_noise, set_uncertainty_service);
        }
        std::function<EigenHelpers::VectorAffine3d(const Eigen::Affine3d&, const Eigen::Affine3d&, const double, const double, const bool)> robot_execution_fn = [&] (const Eigen::Affine3d& target_configuration, const Eigen::Affine3d& expected_result_configuration, const double duration, const double execution_shortcut_distance, const bool reset) { return move_robot(target_configuration, expected_result_configuration, duration, execution_shortcut_distance, reset, robot_control_service); };
        const auto policy_execution_results = uncertainty_planning_core::ExecuteSE3UncertaintyPolicy(options, robot, simulator, sampler, policy, start_and_goal.first, start_and_goal.second, robot_execution_fn, display_debug_publisher);
        const std::map<std::string, double> policy_execution_stats = policy_execution_results.second.first;
        const std::vector<int64_t> policy_execution_step_counts = policy_execution_results.second.second.first;
        const std::vector<double> policy_execution_times = policy_execution_results.second.second.second;
        std::cout << "Policy execution success: " << PrettyPrint::PrettyPrint(policy_execution_stats) << std::endl;
        complete_policy_stats.insert(policy_execution_stats.begin(), policy_execution_stats.end());
        // Save the executed policy
        uncertainty_planning_core::SaveSE3Policy(policy_execution_results.first, options.executed_policy_file);
        // Print out the results & save them to the log file
        const std::string log_results = "++++++++++\n" + PrettyPrint::PrettyPrint(options) + "\n" + PrettyPrint::PrettyPrint(extra_options) + "\nRESULTS:\n" + PrettyPrint::PrettyPrint(complete_policy_stats, false, "\n") + "\nSimulation step counts: " + PrettyPrint::PrettyPrint(policy_simulation_step_counts) + "\nExecution step counts: " + PrettyPrint::PrettyPrint(policy_execution_step_counts) + "\nExecution times: " + PrettyPrint::PrettyPrint(policy_execution_times);
        std::cout << "Policy results:\n" << log_results << std::endl;
        std::ofstream log_file(options.policy_log_file, std::ios_base::out | std::ios_base::app);
        if (!log_file.is_open())
        {
            std::cerr << "\x1b[31;1m Unable to create folder/file to log to: " << options.policy_log_file << "\x1b[37m \n";
            throw std::invalid_argument( "Log filename must be write-openable" );
        }
        log_file << log_results << std::endl;
        log_file.close();
    }
    catch (...)
    {
        std::cout << "!!! Policy file does not exist, skipping !!!" << std::endl;
        const std::string log_results = "++++++++++\n" + PrettyPrint::PrettyPrint(options) + "\n" + PrettyPrint::PrettyPrint(extra_options) + "\nRESULTS:\n(Execution) Policy success: 0\n(Simulation) Policy success: 0\n(Simulation) Policy successful simulator resolves: 0\n(Simulation) Policy unsuccessful simulator environment resolves: 0\n(Simulation) Policy unsuccessful simulator resolves: 0\n(Simulation) Policy unsuccessful simulator self-collision resolves: 0\nSimulation step counts: -1\nExecution step counts: -1\nExecution times: -0.0";
        std::ofstream log_file(options.policy_log_file, std::ios_base::out | std::ios_base::app);
        if (!log_file.is_open())
        {
            std::cerr << "\x1b[31;1m Unable to create folder/file to log to: " << options.policy_log_file << "\x1b[37m \n";
            throw std::invalid_argument( "Log filename must be write-openable" );
        }
        log_file << log_results << std::endl;
        log_file.close();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "se3_contact_execution_node");
    ros::NodeHandle nh;
    ROS_INFO("Starting SE(3) Contact Execution Node...");
    ros::Publisher display_debug_publisher = nh.advertise<visualization_msgs::MarkerArray>("uncertainty_planning_debug_display_markers", 1, true);
    ros::ServiceClient robot_control_service = nh.serviceClient<uncertainty_planning_examples::Simple6dofRobotMove>("simple_6dof_robot_move");
    ros::ServiceClient set_uncertainty_service = nh.serviceClient<uncertainty_planning_examples::SetSimple6dofActuationError>("simple6dof_robot/set_actuation_uncertainty");
    peg_in_hole_env_se3(display_debug_publisher, robot_control_service, set_uncertainty_service);
    return 0;
}
