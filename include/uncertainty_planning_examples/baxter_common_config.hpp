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
#include "arc_utilities/eigen_helpers.hpp"
#include "arc_utilities/eigen_helpers_conversions.hpp"
#include "arc_utilities/pretty_print.hpp"
#include "arc_utilities/voxel_grid.hpp"
#include "arc_utilities/simple_rrt_planner.hpp"
#include "uncertainty_planning_core/simple_pid_controller.hpp"
#include "uncertainty_planning_core/simple_uncertainty_models.hpp"
#include "uncertainty_planning_core/uncertainty_contact_planning.hpp"
#include "uncertainty_planning_core/simple_robot_models.hpp"
#include "uncertainty_planning_core/simple_samplers.hpp"
#include "uncertainty_planning_core/uncertainty_planning_core.hpp"
#include "fast_kinematic_simulator/fast_kinematic_simulator.hpp"
#include "fast_kinematic_simulator/simulator_environment_builder.hpp"
#include "uncertainty_planning_examples/config_common.hpp"

#ifndef BAXTER_LINKED_COMMON_CONFIG_HPP
#define BAXTER_LINKED_COMMON_CONFIG_HPP

namespace baxter_linked_common_config
{
    typedef simple_robot_models::SimpleJointModel SJM;
    typedef simple_robot_models::SimpleLinkedConfiguration SLC;

    inline uncertainty_planning_core::PLANNING_AND_EXECUTION_OPTIONS GetDefaultOptions()
    {
        uncertainty_planning_core::PLANNING_AND_EXECUTION_OPTIONS options;
        options.clustering_type = uncertainty_contact_planning::CONVEX_REGION_SIGNATURE;
        options.planner_time_limit = 600.0;
        options.goal_bias = 0.15;
        options.step_size = 1.0;
        options.step_duration = 10.0;
        options.goal_probability_threshold = 0.51;
        options.goal_distance_threshold = 0.15; // 0.05;
        options.connect_after_first_solution = 0.25;
        options.signature_matching_threshold = 0.1;
        options.distance_clustering_threshold = M_PI_2;
        options.feasibility_alpha = 0.75;
        options.variance_alpha = 0.75;
        options.edge_attempt_count = 50u;
        options.num_particles = 1u;
        options.use_contact = true;
        options.use_reverse = true;
        options.use_spur_actions = true;
        options.max_exec_actions = 1000u;
        options.max_policy_exec_time = 300.0;
        options.num_policy_simulations = 1u;
        options.num_policy_executions = 1u;
        options.policy_action_attempt_count = 100u;
        options.debug_level = 0u;
        options.planner_log_file = "/tmp/baxter_planner_log.txt";
        options.policy_log_file = "/tmp/baxter_policy_log.txt";
        options.planned_policy_file = "/tmp/baxter_planned_policy.policy";
        options.executed_policy_file = "/dev/null";
        return options;
    }

    inline uncertainty_planning_core::PLANNING_AND_EXECUTION_OPTIONS GetOptions()
    {
        return uncertainty_planning_core::GetOptions(GetDefaultOptions());
    }

    inline config_common::EXTRA_CONFIG_PARAMS GetDefaultExtraOptions()
    {
        return config_common::EXTRA_CONFIG_PARAMS(0.03125, 25.0, 0.0, 0.0, "baxter_blocked_test_mod_env");
    }

    inline config_common::EXTRA_CONFIG_PARAMS GetExtraOptions()
    {
        return config_common::GetOptions(GetDefaultExtraOptions());
    }

    inline simple_robot_models::LINKED_ROBOT_CONFIG GetDefaultRobotConfig(const config_common::EXTRA_CONFIG_PARAMS& options)
    {
        const double env_resolution = options.environment_resolution;
        const double kp = 1.1;
        const double ki = 0.0;
        const double kd = 0.1;
        const double i_clamp = 0.0;
        const double velocity_limit = env_resolution * 2.0;
        const double max_sensor_noise = options.sensor_error;
        const double max_actuator_noise = options.actuator_error;
        const simple_robot_models::LINKED_ROBOT_CONFIG robot_config(kp, ki, kd, i_clamp, velocity_limit, max_sensor_noise, max_actuator_noise);
        return robot_config;
    }

    inline Eigen::Affine3d GetBaseTransform()
    {
        const Eigen::Affine3d base_transform = Eigen::Translation3d(0.0, 0.0, 0.0) * Eigen::Quaterniond(Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()));
        return base_transform;
    }

    inline SLC MakeBaxterLeftArmConfiguration(const std::vector<double>& joint_values)
    {
        assert(joint_values.size() == 7);
        SLC left_arm_configuration(7);
        const double left_s0 = joint_values[0];
        left_arm_configuration[0] = SJM(std::pair<double, double>(-1.70167993878, 1.70167993878), left_s0, SJM::REVOLUTE); // left_s0
        const double left_s1 = joint_values[1];
        left_arm_configuration[1] = SJM(std::pair<double, double>(-2.147, 1.047), left_s1, SJM::REVOLUTE); // left_s1
        const double left_e0 = joint_values[2];
        left_arm_configuration[2] = SJM(std::pair<double, double>(-3.05417993878, 3.05417993878), left_e0, SJM::REVOLUTE); // left_e0
        const double left_e1 = joint_values[3];
        left_arm_configuration[3] = SJM(std::pair<double, double>(-0.05, 2.618), left_e1, SJM::REVOLUTE); // left_e1
        const double left_w0 = joint_values[4];
        left_arm_configuration[4] = SJM(std::pair<double, double>(-3.059, 3.059), left_w0, SJM::REVOLUTE); // left_w0
        const double left_w1 = joint_values[5];
        left_arm_configuration[5] = SJM(std::pair<double, double>(-1.57079632679, 2.094), left_w1, SJM::REVOLUTE); // left_w1
        const double left_w2 = joint_values[6];
        left_arm_configuration[6] = SJM(std::pair<double, double>(-3.059, 3.059), left_w2, SJM::REVOLUTE); // left_w2
        return left_arm_configuration;
    }

    inline SLC MakeBaxterRightArmConfiguration(const std::vector<double>& joint_values)
    {
        assert(joint_values.size() == 7);
        SLC right_arm_configuration(7);
        const double right_s0 = joint_values[0];
        right_arm_configuration[0] = SJM(std::pair<double, double>(-1.70167993878, 1.70167993878), right_s0, SJM::REVOLUTE); // right_s0
        const double right_s1 = joint_values[1];
        right_arm_configuration[1] = SJM(std::pair<double, double>(-2.147, 1.047), right_s1, SJM::REVOLUTE); // right_s1
        const double right_e0 = joint_values[2];
        right_arm_configuration[2] = SJM(std::pair<double, double>(-3.05417993878, 3.05417993878), right_e0, SJM::REVOLUTE); // right_e0
        const double right_e1 = joint_values[3];
        right_arm_configuration[3] = SJM(std::pair<double, double>(-0.05, 2.618), right_e1, SJM::REVOLUTE); // right_e1
        const double right_w0 = joint_values[4];
        right_arm_configuration[4] = SJM(std::pair<double, double>(-3.059, 3.059), right_w0, SJM::REVOLUTE); // right_w0
        const double right_w1 = joint_values[5];
        right_arm_configuration[5] = SJM(std::pair<double, double>(-1.57079632679, 2.094), right_w1, SJM::REVOLUTE); // right_w1
        const double right_w2 = joint_values[6];
        right_arm_configuration[6] = SJM(std::pair<double, double>(-3.059, 3.059), right_w2, SJM::REVOLUTE); // right_w2
        return right_arm_configuration;
    }

    inline std::pair<SLC, SLC> GetStartAndGoal()
    {
        // Define the goals of the plan
        const SLC goal = MakeBaxterRightArmConfiguration(std::vector<double>{0.5821457090025145, -0.27496605622846043, 0.016490293469768196, 1.1508690861110316, -0.24045148850103862, -0.89, 0.06366020269724466});
        const SLC start = MakeBaxterRightArmConfiguration(std::vector<double>{0.119267, 1.047, -0.27535, -0.05, -1.29315, -1.218, -0.29184});
        return std::make_pair(start, goal);
    }

    inline SLC GetReferenceConfiguration()
    {
        const SLC reference_configuration = MakeBaxterRightArmConfiguration(std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        return reference_configuration;
    }

    inline std::vector<double> GetJointUncertaintyParams(const config_common::EXTRA_CONFIG_PARAMS& options)
    {
        const std::vector<double> uncertainty_params(7, options.actuator_error);
        return uncertainty_params;
    }

    inline std::vector<double> GetJointDistanceWeights()
    {
        const std::vector<double> max_velocities = {0.27, 0.27, 0.27, 0.27, 0.3, 0.3, 0.5};
        std::vector<double> distance_weights(max_velocities.size(), 0.0);
        for (size_t idx = 0; idx < max_velocities.size(); idx++)
        {
            distance_weights[idx] = 1.0 / max_velocities[idx];
        }
        return distance_weights;
    }

    inline void GetEndEffectorPoints(simple_robot_models::RobotLink& end_effector_link, const std::string& environment_id)
    {
        if (environment_id == "baxter_wrist_key_env")
        {
            // Key
            end_effector_link.link_points->push_back(Eigen::Vector3d(0.0, 0.0, 0.175));
            end_effector_link.link_points->push_back(Eigen::Vector3d(0.0, 0.0, 0.2));
            end_effector_link.link_points->push_back(Eigen::Vector3d(0.0, 0.0, 0.225));
            end_effector_link.link_points->push_back(Eigen::Vector3d(0.0, 0.0, 0.25));
            //
            end_effector_link.link_points->push_back(Eigen::Vector3d(0.0, 0.0, 0.275));
            end_effector_link.link_points->push_back(Eigen::Vector3d(0.0, 0.0, 0.3));
            end_effector_link.link_points->push_back(Eigen::Vector3d(0.0, 0.0, 0.325));
            end_effector_link.link_points->push_back(Eigen::Vector3d(0.0, 0.0, 0.35));
            //
            end_effector_link.link_points->push_back(Eigen::Vector3d(0.0, 0.025, 0.275));
            end_effector_link.link_points->push_back(Eigen::Vector3d(0.0, 0.025, 0.3));
            end_effector_link.link_points->push_back(Eigen::Vector3d(0.0, 0.025, 0.325));
            end_effector_link.link_points->push_back(Eigen::Vector3d(0.0, 0.025, 0.35));
            //
            end_effector_link.link_points->push_back(Eigen::Vector3d(0.0, 0.05, 0.275));
            end_effector_link.link_points->push_back(Eigen::Vector3d(0.0, 0.05, 0.3));
            end_effector_link.link_points->push_back(Eigen::Vector3d(0.0, 0.05, 0.325));
            end_effector_link.link_points->push_back(Eigen::Vector3d(0.0, 0.05, 0.35));
            //
            end_effector_link.link_points->push_back(Eigen::Vector3d(0.0, 0.075, 0.275));
            end_effector_link.link_points->push_back(Eigen::Vector3d(0.0, 0.075, 0.3));
            end_effector_link.link_points->push_back(Eigen::Vector3d(0.0, 0.075, 0.325));
            end_effector_link.link_points->push_back(Eigen::Vector3d(0.0, 0.075, 0.35));
            //
            end_effector_link.link_points->push_back(Eigen::Vector3d(0.025, 0.0, 0.275));
            end_effector_link.link_points->push_back(Eigen::Vector3d(0.025, 0.0, 0.3));
            end_effector_link.link_points->push_back(Eigen::Vector3d(0.025, 0.0, 0.325));
            end_effector_link.link_points->push_back(Eigen::Vector3d(0.025, 0.0, 0.35));
            //
            end_effector_link.link_points->push_back(Eigen::Vector3d(0.05, 0.0, 0.275));
            end_effector_link.link_points->push_back(Eigen::Vector3d(0.05, 0.0, 0.3));
            end_effector_link.link_points->push_back(Eigen::Vector3d(0.05, 0.0, 0.325));
            end_effector_link.link_points->push_back(Eigen::Vector3d(0.05, 0.0, 0.35));
            //
            end_effector_link.link_points->push_back(Eigen::Vector3d(0.075, 0.0, 0.275));
            end_effector_link.link_points->push_back(Eigen::Vector3d(0.075, 0.0, 0.3));
            end_effector_link.link_points->push_back(Eigen::Vector3d(0.075, 0.0, 0.325));
            end_effector_link.link_points->push_back(Eigen::Vector3d(0.075, 0.0, 0.35));
        }
        else if (environment_id == "baxter_blocked_test_env")
        {
            // Peg
            end_effector_link.link_points->push_back(Eigen::Vector3d(0.0, 0.0, 0.175));
            end_effector_link.link_points->push_back(Eigen::Vector3d(0.0, 0.0, 0.2));
            end_effector_link.link_points->push_back(Eigen::Vector3d(0.0, 0.0, 0.225));
            end_effector_link.link_points->push_back(Eigen::Vector3d(0.0, 0.0, 0.24));
        }
        else
        {
            // Peg
            end_effector_link.link_points->push_back(Eigen::Vector3d(0.0, 0.0, 0.175));
            end_effector_link.link_points->push_back(Eigen::Vector3d(0.0, 0.0, 0.2));
            end_effector_link.link_points->push_back(Eigen::Vector3d(0.0, 0.0, 0.225));
            end_effector_link.link_points->push_back(Eigen::Vector3d(0.0, 0.0, 0.25));
        }
    }

    typedef uncertainty_planning_core::LinkedActuatorModel BaxterJointActuatorModel;

    inline simple_robot_models::SimpleLinkedRobot<BaxterJointActuatorModel> GetRobot(const Eigen::Affine3d& base_transform, const simple_robot_models::LINKED_ROBOT_CONFIG& joint_config, const std::vector<double>& joint_uncertainty_params, const std::vector<double>& joint_distance_weights, const std::string& environment_id)
    {
        const double s0_noise = joint_uncertainty_params[0];
        const double s1_noise = joint_uncertainty_params[1];
        const double e0_noise = joint_uncertainty_params[2];
        const double e1_noise = joint_uncertainty_params[3];
        const double w0_noise = joint_uncertainty_params[4];
        const double w1_noise = joint_uncertainty_params[5];
        const double w2_noise = joint_uncertainty_params[6];
        // Make the reference configuration
        const SLC reference_configuration = GetReferenceConfiguration();
        // Make the robot model
        simple_robot_models::RobotLink torso;
        torso.link_name = "torso";
        torso.link_points->push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
        torso.link_points->push_back(Eigen::Vector3d(0.025, 0.0, 0.0));
        torso.link_points->push_back(Eigen::Vector3d(0.0, 0.025, 0.0));
        torso.link_points->push_back(Eigen::Vector3d(0.0, 0.0, 0.025));
        torso.link_points->push_back(Eigen::Vector3d(0.0, -0.025, 0.0));
        torso.link_points->push_back(Eigen::Vector3d(0.0, -0.05, 0.0));
        torso.link_points->push_back(Eigen::Vector3d(0.0, -0.075, 0.0));
        torso.link_points->push_back(Eigen::Vector3d(0.0, -0.1, 0.0));
        torso.link_points->push_back(Eigen::Vector3d(0.0, -0.125, 0.0));
        torso.link_points->push_back(Eigen::Vector3d(0.0, -0.15, 0.0));
        torso.link_points->push_back(Eigen::Vector3d(0.0, -0.175, 0.0));
        torso.link_points->push_back(Eigen::Vector3d(0.0, -0.2, 0.0));
        torso.link_points->push_back(Eigen::Vector3d(0.0, -0.225, 0.0));
        torso.link_points->push_back(Eigen::Vector3d(0.0, -0.225, 0.025));
        torso.link_points->push_back(Eigen::Vector3d(0.0, -0.225, 0.05));
        torso.link_points->push_back(Eigen::Vector3d(0.0, -0.225, 0.075));
        torso.link_points->push_back(Eigen::Vector3d(0.0, -0.225, 0.1));
        torso.link_points->push_back(Eigen::Vector3d(0.025, -0.225, 0.1));
        simple_robot_models::RobotLink right_arm_mount;
        right_arm_mount.link_name = "right_arm_mount";
        right_arm_mount.link_points->push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
        right_arm_mount.link_points->push_back(Eigen::Vector3d(0.025, 0.0, 0.0));
        right_arm_mount.link_points->push_back(Eigen::Vector3d(0.05, 0.0, 0.0));
        simple_robot_models::RobotLink right_upper_shoulder;
        right_upper_shoulder.link_name = "right_upper_shoulder";
        right_upper_shoulder.link_points->push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
        right_upper_shoulder.link_points->push_back(Eigen::Vector3d(0.0, 0.0, 0.025));
        right_upper_shoulder.link_points->push_back(Eigen::Vector3d(0.0, 0.0, 0.05));
        right_upper_shoulder.link_points->push_back(Eigen::Vector3d(0.0, 0.0, 0.075));
        right_upper_shoulder.link_points->push_back(Eigen::Vector3d(0.0, 0.0, 0.1));
        right_upper_shoulder.link_points->push_back(Eigen::Vector3d(0.0, 0.0, 0.125));
        right_upper_shoulder.link_points->push_back(Eigen::Vector3d(0.0, 0.0, 0.15));
        right_upper_shoulder.link_points->push_back(Eigen::Vector3d(0.0, 0.0, 0.175));
        right_upper_shoulder.link_points->push_back(Eigen::Vector3d(0.0, 0.0, 0.2));
        right_upper_shoulder.link_points->push_back(Eigen::Vector3d(0.0, 0.0, 0.225));
        right_upper_shoulder.link_points->push_back(Eigen::Vector3d(0.0, 0.0, 0.25));
        right_upper_shoulder.link_points->push_back(Eigen::Vector3d(0.0, 0.0, 0.275));
        right_upper_shoulder.link_points->push_back(Eigen::Vector3d(0.025, 0.0, 0.275));
        right_upper_shoulder.link_points->push_back(Eigen::Vector3d(0.05, 0.0, 0.275));
        simple_robot_models::RobotLink right_lower_shoulder;
        right_lower_shoulder.link_name = "right_lower_shoulder";
        right_lower_shoulder.link_points->push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
        right_lower_shoulder.link_points->push_back(Eigen::Vector3d(0.025, 0.0, 0.0));
        right_lower_shoulder.link_points->push_back(Eigen::Vector3d(0.05, 0.0, 0.0));
        right_lower_shoulder.link_points->push_back(Eigen::Vector3d(0.075, 0.0, 0.0));
        right_lower_shoulder.link_points->push_back(Eigen::Vector3d(0.1, 0.0, 0.0));
        simple_robot_models::RobotLink right_upper_elbow;
        right_upper_elbow.link_name = "right_upper_elbow";
        right_upper_elbow.link_points->push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
        right_upper_elbow.link_points->push_back(Eigen::Vector3d(0.0, 0.0, 0.025));
        right_upper_elbow.link_points->push_back(Eigen::Vector3d(0.0, 0.0, 0.05));
        right_upper_elbow.link_points->push_back(Eigen::Vector3d(0.0, 0.0, 0.075));
        right_upper_elbow.link_points->push_back(Eigen::Vector3d(0.0, 0.0, 0.1));
        right_upper_elbow.link_points->push_back(Eigen::Vector3d(0.0, 0.0, 0.125));
        right_upper_elbow.link_points->push_back(Eigen::Vector3d(0.0, 0.0, 0.15));
        right_upper_elbow.link_points->push_back(Eigen::Vector3d(0.0, 0.0, 0.175));
        right_upper_elbow.link_points->push_back(Eigen::Vector3d(0.0, 0.0, 0.2));
        right_upper_elbow.link_points->push_back(Eigen::Vector3d(0.0, 0.0, 0.225));
        right_upper_elbow.link_points->push_back(Eigen::Vector3d(0.0, 0.0, 0.25));
        right_upper_elbow.link_points->push_back(Eigen::Vector3d(0.025, 0.0, 0.25));
        right_upper_elbow.link_points->push_back(Eigen::Vector3d(0.05, 0.0, 0.25));
        simple_robot_models::RobotLink right_lower_elbow;
        right_lower_elbow.link_name = "right_lower_elbow";
        right_lower_elbow.link_points->push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
        right_lower_elbow.link_points->push_back(Eigen::Vector3d(0.025, 0.0, 0.0));
        right_lower_elbow.link_points->push_back(Eigen::Vector3d(0.05, 0.0, 0.0));
        right_lower_elbow.link_points->push_back(Eigen::Vector3d(0.075, 0.0, 0.0));
        right_lower_elbow.link_points->push_back(Eigen::Vector3d(0.1, 0.0, 0.0));
        simple_robot_models::RobotLink right_upper_forearm;
        right_upper_forearm.link_name = "right_upper_forearm";
        right_upper_forearm.link_points->push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
        right_upper_forearm.link_points->push_back(Eigen::Vector3d(0.0, 0.0, 0.025));
        right_upper_forearm.link_points->push_back(Eigen::Vector3d(0.0, 0.0, 0.05));
        right_upper_forearm.link_points->push_back(Eigen::Vector3d(0.0, 0.0, 0.075));
        right_upper_forearm.link_points->push_back(Eigen::Vector3d(0.0, 0.0, 0.1));
        right_upper_forearm.link_points->push_back(Eigen::Vector3d(0.0, 0.0, 0.125));
        right_upper_forearm.link_points->push_back(Eigen::Vector3d(0.0, 0.0, 0.15));
        right_upper_forearm.link_points->push_back(Eigen::Vector3d(0.0, 0.0, 0.175));
        right_upper_forearm.link_points->push_back(Eigen::Vector3d(0.0, 0.0, 0.2));
        right_upper_forearm.link_points->push_back(Eigen::Vector3d(0.0, 0.0, 0.225));
        right_upper_forearm.link_points->push_back(Eigen::Vector3d(0.0, 0.0, 0.25));
        right_upper_forearm.link_points->push_back(Eigen::Vector3d(0.0, 0.0, 0.275));
        simple_robot_models::RobotLink right_lower_forearm;
        right_lower_forearm.link_name = "right_lower_forearm";
        right_lower_forearm.link_points->push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
        right_lower_forearm.link_points->push_back(Eigen::Vector3d(0.025, 0.0, 0.0));
        right_lower_forearm.link_points->push_back(Eigen::Vector3d(0.05, 0.0, 0.0));
        right_lower_forearm.link_points->push_back(Eigen::Vector3d(0.075, 0.0, 0.0));
        right_lower_forearm.link_points->push_back(Eigen::Vector3d(0.1, 0.0, 0.0));
        right_lower_forearm.link_points->push_back(Eigen::Vector3d(0.125, 0.0, 0.0));
        simple_robot_models::RobotLink right_wrist;
        right_wrist.link_name = "right_wrist";
        right_wrist.link_points->push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
        right_wrist.link_points->push_back(Eigen::Vector3d(0.0, 0.0, 0.025));
        right_wrist.link_points->push_back(Eigen::Vector3d(0.0, 0.0, 0.05));
        right_wrist.link_points->push_back(Eigen::Vector3d(0.0, 0.0, 0.075));
        right_wrist.link_points->push_back(Eigen::Vector3d(0.0, 0.0, 0.1));
        right_wrist.link_points->push_back(Eigen::Vector3d(0.0, 0.0, 0.125));
        right_wrist.link_points->push_back(Eigen::Vector3d(0.0, 0.0, 0.15));
        // Get the end of the wrist/EE shape
        GetEndEffectorPoints(right_wrist, environment_id);
        const std::vector<simple_robot_models::RobotLink> links = {torso, right_arm_mount, right_upper_shoulder, right_lower_shoulder, right_upper_elbow, right_lower_elbow, right_upper_forearm, right_lower_forearm, right_wrist};
        const std::vector<std::pair<size_t, size_t>> allowed_self_collisions = {std::pair<size_t, size_t>(0, 1), std::pair<size_t, size_t>(1, 2), std::pair<size_t, size_t>(2, 3), std::pair<size_t, size_t>(3, 4), std::pair<size_t, size_t>(4, 5), std::pair<size_t, size_t>(5, 6), std::pair<size_t, size_t>(6, 7), std::pair<size_t, size_t>(7, 8)};
        // right_s0
        simple_robot_models::RobotJoint<BaxterJointActuatorModel> right_arm_mount_joint;
        right_arm_mount_joint.name = "right_arm_mount_joint";
        right_arm_mount_joint.parent_link_index = 0;
        right_arm_mount_joint.child_link_index = 1;
        right_arm_mount_joint.joint_axis = Eigen::Vector3d::UnitZ();
        right_arm_mount_joint.joint_transform = Eigen::Translation3d(0.024645, -0.219645, 0.118588) * EigenHelpers::QuaternionFromUrdfRPY(0.0, 0.0, -0.7854);
        right_arm_mount_joint.joint_model = SJM(std::make_pair(0.0, 0.0), 0.0, SJM::FIXED);
        // We don't need an uncertainty model for a fixed joint
        right_arm_mount_joint.joint_controller = simple_robot_models::JointControllerGroup<BaxterJointActuatorModel>(joint_config);
        // right_s0
        simple_robot_models::RobotJoint<BaxterJointActuatorModel> right_s0;
        right_s0.name = "right_s0";
        right_s0.parent_link_index = 1;
        right_s0.child_link_index = 2;
        right_s0.joint_axis = Eigen::Vector3d::UnitZ();
        right_s0.joint_transform = Eigen::Translation3d(0.055695, 0.0, 0.011038) * EigenHelpers::QuaternionFromUrdfRPY(0.0, 0.0, 0.0);
        right_s0.joint_model = reference_configuration[0];
        const BaxterJointActuatorModel right_s0_joint_model(std::abs(s0_noise), 0.5);
        simple_robot_models::LINKED_ROBOT_CONFIG s0_config = joint_config;
        s0_config.velocity_limit = 0.27 * 0.75;
        right_s0.joint_controller = simple_robot_models::JointControllerGroup<BaxterJointActuatorModel>(s0_config, right_s0_joint_model);
        // Base pitch
        simple_robot_models::RobotJoint<BaxterJointActuatorModel> right_s1;
        right_s1.name = "right_s1";
        right_s1.parent_link_index = 2;
        right_s1.child_link_index = 3;
        right_s1.joint_axis = Eigen::Vector3d::UnitZ();
        right_s1.joint_transform = Eigen::Translation3d(0.069, 0.0, 0.27035) * EigenHelpers::QuaternionFromUrdfRPY(-1.57079632679, 0.0, 0.0);
        right_s1.joint_model = reference_configuration[1];
        const BaxterJointActuatorModel right_s1_joint_model(std::abs(s1_noise), 0.5);
        simple_robot_models::LINKED_ROBOT_CONFIG s1_config = joint_config;
        s1_config.velocity_limit = 0.27 * 0.75;
        right_s1.joint_controller = simple_robot_models::JointControllerGroup<BaxterJointActuatorModel>(s1_config, right_s1_joint_model);
        // Elbow pitch
        simple_robot_models::RobotJoint<BaxterJointActuatorModel> right_e0;
        right_e0.name = "right_e0";
        right_e0.parent_link_index = 3;
        right_e0.child_link_index = 4;
        right_e0.joint_axis = Eigen::Vector3d::UnitZ();
        right_e0.joint_transform = Eigen::Translation3d(0.102, 0.0, 0.0) * EigenHelpers::QuaternionFromUrdfRPY(1.57079632679, 0.0, 1.57079632679);
        right_e0.joint_model = reference_configuration[2];
        const BaxterJointActuatorModel right_e0_joint_model(std::abs(e0_noise), 0.5);
        simple_robot_models::LINKED_ROBOT_CONFIG e0_config = joint_config;
        e0_config.velocity_limit = 0.27 * 0.75;
        right_e0.joint_controller = simple_robot_models::JointControllerGroup<BaxterJointActuatorModel>(e0_config, right_e0_joint_model);
        // Elbow roll
        simple_robot_models::RobotJoint<BaxterJointActuatorModel> right_e1;
        right_e1.name = "right_e1";
        right_e1.parent_link_index = 4;
        right_e1.child_link_index = 5;
        right_e1.joint_axis = Eigen::Vector3d::UnitZ();
        right_e1.joint_transform = Eigen::Translation3d(0.069, 0.0, 0.26242) * EigenHelpers::QuaternionFromUrdfRPY(-1.57079632679, -1.57079632679, 0.0);
        right_e1.joint_model = reference_configuration[3];
        const BaxterJointActuatorModel right_e1_joint_model(std::abs(e1_noise), 0.5);
        simple_robot_models::LINKED_ROBOT_CONFIG e1_config = joint_config;
        e1_config.velocity_limit = 0.27 * 0.75;
        right_e1.joint_controller = simple_robot_models::JointControllerGroup<BaxterJointActuatorModel>(e1_config, right_e1_joint_model);
        // Wrist pitch
        simple_robot_models::RobotJoint<BaxterJointActuatorModel> right_w0;
        right_w0.name = "right_w0";
        right_w0.parent_link_index = 5;
        right_w0.child_link_index = 6;
        right_w0.joint_axis = Eigen::Vector3d::UnitZ();
        right_w0.joint_transform = Eigen::Translation3d(0.10359, 0.0, 0.0) * EigenHelpers::QuaternionFromUrdfRPY(1.57079632679, 0.0, 1.57079632679);
        right_w0.joint_model = reference_configuration[4];
        const BaxterJointActuatorModel right_w0_joint_model(std::abs(w0_noise), 1.0);
        simple_robot_models::LINKED_ROBOT_CONFIG w0_config = joint_config;
        w0_config.velocity_limit = 0.3 * 0.75;
        right_w0.joint_controller = simple_robot_models::JointControllerGroup<BaxterJointActuatorModel>(w0_config, right_w0_joint_model);
        // Wrist roll
        simple_robot_models::RobotJoint<BaxterJointActuatorModel> right_w1;
        right_w1.name = "right_w1";
        right_w1.parent_link_index = 6;
        right_w1.child_link_index = 7;
        right_w1.joint_axis = Eigen::Vector3d::UnitZ();
        right_w1.joint_transform = Eigen::Translation3d(0.01, 0.0, 0.2707) * EigenHelpers::QuaternionFromUrdfRPY(-1.57079632679, -1.57079632679, 0.0);
        right_w1.joint_model = reference_configuration[5];
        const BaxterJointActuatorModel right_w1_joint_model(std::abs(w1_noise), 1.0);
        simple_robot_models::LINKED_ROBOT_CONFIG w1_config = joint_config;
        w1_config.velocity_limit = 0.3 * 0.75;
        right_w1.joint_controller = simple_robot_models::JointControllerGroup<BaxterJointActuatorModel>(w1_config, right_w1_joint_model);
        // Wrist roll
        simple_robot_models::RobotJoint<BaxterJointActuatorModel> right_w2;
        right_w2.name = "right_w2";
        right_w2.parent_link_index = 7;
        right_w2.child_link_index = 8;
        right_w2.joint_axis = Eigen::Vector3d::UnitZ();
        right_w2.joint_transform = Eigen::Translation3d(0.115975, 0.0, 0.0) * EigenHelpers::QuaternionFromUrdfRPY(1.57079632679, 0.0, 1.57079632679);
        right_w2.joint_model = reference_configuration[6];
        const BaxterJointActuatorModel right_w2_joint_model(std::abs(w2_noise), 1.0);
        simple_robot_models::LINKED_ROBOT_CONFIG w2_config = joint_config;
        w2_config.velocity_limit = 0.5 * 0.75;
        right_w2.joint_controller = simple_robot_models::JointControllerGroup<BaxterJointActuatorModel>(w2_config, right_w2_joint_model);
        const std::vector<simple_robot_models::RobotJoint<BaxterJointActuatorModel>> joints = {right_arm_mount_joint, right_s0, right_s1, right_e0, right_e1, right_w0, right_w1, right_w2};
        const simple_robot_models::SimpleLinkedRobot<BaxterJointActuatorModel> robot(base_transform, links, joints, allowed_self_collisions, reference_configuration, joint_distance_weights);
        return robot;
    }

    inline uncertainty_planning_core::LinkedSamplerPtr GetSampler()
    {
        // Make the sampler
        const SLC reference_configuration = GetReferenceConfiguration();
        return uncertainty_planning_core::LinkedSamplerPtr(new simple_samplers::SimpleLinkedBaseSampler<uncertainty_planning_core::PRNG>(reference_configuration));
    }

    inline uncertainty_planning_core::LinkedSimulatorPtr GetSimulator(const config_common::EXTRA_CONFIG_PARAMS& options, const int32_t debug_level)
    {
        const simulator_environment_builder::EnvironmentComponents environment_components = simulator_environment_builder::BuildCompleteEnvironment(options.environment_id, options.environment_resolution);
        const fast_kinematic_simulator::SolverParameters solver_params = fast_kinematic_simulator::GetDefaultSolverParameters();
        return fast_kinematic_simulator::MakeLinkedSimulator(environment_components.GetEnvironment(), environment_components.GetEnvironmentSDF(), environment_components.GetSurfaceNormalsGrid(), solver_params, options.simulation_controller_frequency, debug_level);
    }
}

#endif // BAXTER_LINKED_COMMON_CONFIG_HPP
