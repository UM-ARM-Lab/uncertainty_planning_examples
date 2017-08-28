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
#include <uncertainty_planning_examples/baxter_common_config.hpp>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <uncertainty_planning_examples/SimpleLinkedRobotMove.h>
#include <uncertainty_planning_examples/SetLinkedActuationError.h>

using namespace uncertainty_contact_planning;

inline bool IsSubset(const std::vector<std::string>& candidate_set, const std::vector<std::string>& candidate_subset)
{
    if (candidate_set.size() < candidate_subset.size())
    {
        return false;
    }
    std::map<std::string, uint8_t> names_map;
    for (size_t idx = 0; idx < candidate_set.size(); idx++)
    {
        const std::string& name = candidate_set[idx];
        names_map[name] = 0x01;
    }
    for (size_t idx = 0; idx < candidate_subset.size(); idx++)
    {
        const std::string& name = candidate_subset[idx];
        const auto found_itr = names_map.find(name);
        if (found_itr == names_map.end())
        {
            return false;
        }
    }
    return true;
}

inline bool SetsEqual(const std::vector<std::string>& set1, const std::vector<std::string>& set2)
{
    if (IsSubset(set1, set2) && IsSubset(set2, set1))
    {
        return true;
    }
    else
    {
        return false;
    }
}

inline baxter_linked_common_config::SLC ExtractConfig(const std::vector<std::string>& reference_joint_names, const std::vector<std::string>& current_joint_names, const std::vector<double>& current_joint_positions, const baxter_linked_common_config::SLC& reference_config)
{
    assert(reference_joint_names.size() == reference_config.size());
    assert(current_joint_names.size() == current_joint_positions.size());
    assert(reference_joint_names.size() == current_joint_names.size());
    assert(SetsEqual(reference_joint_names, current_joint_names));
    // Push the joint state into a map
    std::map<std::string, double> current_state_map;
    for (size_t idx = 0; idx < current_joint_names.size(); idx++)
    {
        const std::string& name = current_joint_names[idx];
        const double position = current_joint_positions[idx];
        current_state_map[name] = position;
    }
    // Extract the joint positions in order
    baxter_linked_common_config::SLC current_config;
    current_config.reserve(current_joint_names.size());
    for (size_t idx = 0; idx < reference_joint_names.size(); idx++)
    {
        const std::string& joint_name = reference_joint_names[idx];
        const baxter_linked_common_config::SJM& reference_joint = reference_config[idx];
        const auto found_itr = current_state_map.find(joint_name);
        assert(found_itr != current_state_map.end());
        const double position = found_itr->second;
        current_config.push_back(reference_joint.CopyWithNewValue(position));
    }
    current_config.shrink_to_fit();
    return current_config;
}

inline std::vector<baxter_linked_common_config::SLC, std::allocator<baxter_linked_common_config::SLC>> move_robot(const std::vector<std::string>& joint_names, const baxter_linked_common_config::SLC& target_config, const baxter_linked_common_config::SLC& expected_result_configuration, const double duration, const double execution_shortcut_distance, const bool reset, ros::ServiceClient& robot_control_service)
{
    // Put together service call
    uncertainty_planning_examples::SimpleLinkedRobotMove::Request req;
    assert(joint_names.size() == target_config.size());
    req.joint_name = joint_names;
    req.max_execution_time = ros::Duration(duration);
    req.execution_shortcut_distance = execution_shortcut_distance;
    if (reset)
    {
        std::cout << "Resetting robot to config: " << PrettyPrint::PrettyPrint(joint_names) << ":\n" << PrettyPrint::PrettyPrint(target_config) << std::endl;
        for (size_t idx = 0; idx < target_config.size(); idx++)
        {
            const baxter_linked_common_config::SJM& current_joint = target_config[idx];
            req.start_position.push_back(current_joint.GetValue());
        }
        req.mode = uncertainty_planning_examples::SimpleLinkedRobotMove::Request::RESET;
    }
    else
    {
        std::cout << "Commanding robot to config: " << PrettyPrint::PrettyPrint(joint_names) << ":\n" << PrettyPrint::PrettyPrint(target_config) << std::endl;
        for (size_t idx = 0; idx < target_config.size(); idx++)
        {
            const baxter_linked_common_config::SJM& current_joint = target_config[idx];
            const baxter_linked_common_config::SJM& expected_joint = expected_result_configuration[idx];
            req.target_position.push_back(current_joint.GetValue());
            req.expected_result_position.push_back(expected_joint.GetValue());
        }
        req.mode = uncertainty_planning_examples::SimpleLinkedRobotMove::Request::EXECUTE;
    }
    std::cout << "Calling move service..." << std::endl;
    uncertainty_planning_examples::SimpleLinkedRobotMove::Response res;
    // Call service
    try
    {
        robot_control_service.call(req, res);
    }
    catch (...)
    {
        ROS_ERROR("Move service failed");
    }
    std::cout << "Processing move service response..." << std::endl;
    // Unpack result
    const std::vector<control_msgs::JointTrajectoryControllerState>& trajectory = res.trajectory;
    std::vector<baxter_linked_common_config::SLC, std::allocator<baxter_linked_common_config::SLC>> configs(trajectory.size());
    for (size_t idx = 0; idx < configs.size(); idx++)
    {
        const control_msgs::JointTrajectoryControllerState& current_state = trajectory[idx];
        configs[idx] = ExtractConfig(joint_names, current_state.joint_names, current_state.actual.positions, target_config);
    }
    std::cout << "Reached config: " << PrettyPrint::PrettyPrint(joint_names) << ":\n" << PrettyPrint::PrettyPrint(configs.back()) << "\nafter " << configs.size() << " steps" << std::endl;
    return configs;
}

void set_uncertainty(const std::vector<double>& joint_uncertainties, ros::ServiceClient& set_uncertainty_service)
{
    assert(joint_uncertainties.size() == 7);
    uncertainty_planning_examples::SetLinkedActuationErrorRequest req;
    req.actuator_name = {"right_s0", "right_s1", "right_e0", "right_e1", "right_w0", "right_w1", "right_w2"};
    req.actuator_error = joint_uncertainties;
    uncertainty_planning_examples::SetLinkedActuationErrorResponse res;
    if (set_uncertainty_service.call(req, res) == false)
    {
        std::cout << "Failed to set actuator error" << std::endl;
        throw std::invalid_argument("SetActuationError failed");
    }
}

void execute(ros::Publisher& display_debug_publisher, ros::ServiceClient& robot_control_service, ros::ServiceClient& set_uncertainty_service)
{
    const uncertainty_planning_core::PLANNING_AND_EXECUTION_OPTIONS options = baxter_linked_common_config::GetOptions();
    const config_common::TASK_CONFIG_PARAMS extra_options = baxter_linked_common_config::GetExtraOptions();
    std::cout << PrettyPrint::PrettyPrint(options) << "\n" << PrettyPrint::PrettyPrint(extra_options) << std::endl;
    const std::vector<double> joint_uncertainty_params = baxter_linked_common_config::GetJointUncertaintyParams(extra_options);
    assert(joint_uncertainty_params.size() == 7);
    const std::vector<double> joint_distance_weights = baxter_linked_common_config::GetJointDistanceWeights();
    assert(joint_distance_weights.size() == 7);
    const std::pair<uncertainty_planning_core::LinkedConfig, uncertainty_planning_core::LinkedConfig> start_and_goal = baxter_linked_common_config::GetStartAndGoal();
    const uncertainty_planning_core::LinkedSamplerPtr sampler = baxter_linked_common_config::GetSampler();
    const simple_robot_models::LINKED_ROBOT_CONFIG robot_config = baxter_linked_common_config::GetDefaultRobotConfig(extra_options);
    const Eigen::Isometry3d base_transform = baxter_linked_common_config::GetBaseTransform();
    const uncertainty_planning_core::LinkedRobot robot = baxter_linked_common_config::GetRobot(base_transform, robot_config, joint_uncertainty_params, joint_distance_weights, extra_options.environment_id);
    const uncertainty_planning_core::LinkedSimulatorPtr simulator = baxter_linked_common_config::GetSimulator(extra_options, options.debug_level);
    // Load the policy
    try
    {
        auto policy = uncertainty_planning_core::LoadLinkedPolicy(options.planned_policy_file);
        policy.SetPolicyActionAttemptCount(options.policy_action_attempt_count);
        std::map<std::string, double> complete_policy_stats;
        std::cout << "Press ENTER to simulate policy..." << std::endl;
        //std::cin.get();
        const auto policy_simulation_results = uncertainty_planning_core::SimulateLinkedUncertaintyPolicy(options, robot, simulator, sampler, policy, start_and_goal.first, start_and_goal.second, display_debug_publisher);
        const std::map<std::string, double> policy_simulation_stats = policy_simulation_results.second.first;
        const std::vector<int64_t> policy_simulation_step_counts = policy_simulation_results.second.second.first;
        std::cout << "Policy simulation success: " << PrettyPrint::PrettyPrint(policy_simulation_stats) << std::endl;
        complete_policy_stats.insert(policy_simulation_stats.begin(), policy_simulation_stats.end());
        std::cout << "Press ENTER to execute policy..." << std::endl;
        //std::cin.get();
        if (options.num_policy_executions > 0)
        {
            std::cout << "Setting actuation uncertainty..." << std::endl;
            set_uncertainty(joint_uncertainty_params, set_uncertainty_service);
        }
        const std::vector<std::string> joint_names = robot.GetActiveJointNames();
        std::function<std::vector<baxter_linked_common_config::SLC, std::allocator<baxter_linked_common_config::SLC>>(const baxter_linked_common_config::SLC&, const baxter_linked_common_config::SLC&, const double, const double, const bool)> robot_execution_fn = [&] (const baxter_linked_common_config::SLC& target_configuration, const baxter_linked_common_config::SLC& expected_result_configuration, const double duration, const double execution_shortcut_distance, const bool reset) { return move_robot(joint_names, target_configuration, expected_result_configuration, duration, execution_shortcut_distance, reset, robot_control_service); };
        //options.step_duration = 45.0;
        const auto policy_execution_results = uncertainty_planning_core::ExecuteLinkedUncertaintyPolicy(options, robot, simulator, sampler, policy, start_and_goal.first, start_and_goal.second, robot_execution_fn, display_debug_publisher);
        const std::map<std::string, double> policy_execution_stats = policy_execution_results.second.first;
        const std::vector<int64_t> policy_execution_step_counts = policy_execution_results.second.second.first;
        const std::vector<double> policy_execution_times = policy_execution_results.second.second.second;
        std::cout << "Policy execution success: " << PrettyPrint::PrettyPrint(policy_execution_stats) << std::endl;
        complete_policy_stats.insert(policy_execution_stats.begin(), policy_execution_stats.end());
        // Save the executed policy
        uncertainty_planning_core::SaveLinkedPolicy(policy_execution_results.first, options.executed_policy_file);
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
    ros::init(argc, argv, "linked_contact_execution_node");
    ros::NodeHandle nh;
    ROS_INFO("Starting Baxter Contact Execution Node...");
    ros::Publisher display_debug_publisher = nh.advertise<visualization_msgs::MarkerArray>("uncertainty_planning_debug_display_markers", 1, true);
    ros::ServiceClient robot_control_service = nh.serviceClient<uncertainty_planning_examples::SimpleLinkedRobotMove>("simple_linked_robot_move");
    ros::ServiceClient set_uncertainty_service = nh.serviceClient<uncertainty_planning_examples::SetLinkedActuationError>("baxter_robot/set_actuation_uncertainty");
    //std::cout << set_uncertainty_service.getService() << std::endl;
    //set_uncertainty_service.waitForExistence();
    execute(display_debug_publisher, robot_control_service, set_uncertainty_service);
    return 0;
}
