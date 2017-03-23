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

#ifndef IIWA7_LINKED_COMMON_CONFIG_HPP
#define IIWA7_LINKED_COMMON_CONFIG_HPP

namespace iiwa7_linked_common_config
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
        options.planner_log_file = "/tmp/iiwa7_planner_log.txt";
        options.policy_log_file = "/tmp/iiwa7_policy_log.txt";
        options.planned_policy_file = "/tmp/iiwa7_planned_policy.policy";
        options.executed_policy_file = "/dev/null";
        return options;
    }

    inline uncertainty_planning_core::PLANNING_AND_EXECUTION_OPTIONS GetOptions()
    {
        return uncertainty_planning_core::GetOptions(GetDefaultOptions());
    }

    inline config_common::TASK_CONFIG_PARAMS GetDefaultExtraOptions()
    {
        return config_common::TASK_CONFIG_PARAMS(0.03125, 25.0, 0.0, 0.0, "baxter_blocked_test_mod_env");
    }

    inline config_common::TASK_CONFIG_PARAMS GetExtraOptions()
    {
        return config_common::GetOptions(GetDefaultExtraOptions());
    }

    inline simple_robot_models::LINKED_ROBOT_CONFIG GetDefaultRobotConfig(const config_common::TASK_CONFIG_PARAMS& options)
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

    inline SLC MakeIIWA7ArmConfiguration(const std::vector<double>& joint_values)
    {
        assert(joint_values.size() == 7);
        const double iiwa_joint_1 = joint_values[0];
        const double iiwa_joint_2 = joint_values[1];
        const double iiwa_joint_3 = joint_values[2];
        const double iiwa_joint_4 = joint_values[3];
        const double iiwa_joint_5 = joint_values[4];
        const double iiwa_joint_6 = joint_values[5];
        const double iiwa_joint_7 = joint_values[6];
        SLC arm_configuration(7);
        arm_configuration[0] = SJM(std::make_pair(-2.96705972839, 2.96705972839), iiwa_joint_1, SJM::REVOLUTE);
        arm_configuration[1] = SJM(std::make_pair(-2.09439510239, 2.09439510239), iiwa_joint_2, SJM::REVOLUTE);
        arm_configuration[2] = SJM(std::make_pair(-2.96705972839, 2.96705972839), iiwa_joint_3, SJM::REVOLUTE);
        arm_configuration[3] = SJM(std::make_pair(-2.09439510239, 2.09439510239), iiwa_joint_4, SJM::REVOLUTE);
        arm_configuration[4] = SJM(std::make_pair(-2.96705972839, 2.96705972839), iiwa_joint_5, SJM::REVOLUTE);
        arm_configuration[5] = SJM(std::make_pair(-2.09439510239, 2.09439510239), iiwa_joint_6, SJM::REVOLUTE);
        arm_configuration[6] = SJM(std::make_pair(-3.05432619099, 3.05432619099), iiwa_joint_7, SJM::REVOLUTE);
        return arm_configuration;
    }

    inline std::pair<SLC, SLC> GetStartAndGoal()
    {
        // Define the goals of the plan
        const SLC start = MakeIIWA7ArmConfiguration(std::vector<double>{0.119267, 1.047, -0.27535, -0.05, -1.29315, -1.218, -0.29184});
        const SLC goal = MakeIIWA7ArmConfiguration(std::vector<double>{0.5821457090025145, -0.27496605622846043, 0.016490293469768196, 1.1508690861110316, -0.24045148850103862, -0.89, 0.06366020269724466});
        return std::make_pair(start, goal);
    }

    inline SLC GetReferenceConfiguration()
    {
        const SLC reference_configuration = MakeIIWA7ArmConfiguration(std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        return reference_configuration;
    }

    inline std::vector<double> GetJointUncertaintyParams(const config_common::TASK_CONFIG_PARAMS& options)
    {
        const std::vector<double> uncertainty_params(7, options.actuator_error);
        return uncertainty_params;
    }

    inline std::vector<double> GetMaxVelocities()
    {
        const std::vector<double> max_velocities = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
        return max_velocities;
    }

    inline std::vector<double> GetJointDistanceWeights()
    {
        const std::vector<double> max_velocities = GetMaxVelocities();
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
            end_effector_link.link_points->push_back(Eigen::Vector4d(0.0, 0.0, 0.175, 1.0));
            end_effector_link.link_points->push_back(Eigen::Vector4d(0.0, 0.0, 0.2, 1.0));
            end_effector_link.link_points->push_back(Eigen::Vector4d(0.0, 0.0, 0.225, 1.0));
            end_effector_link.link_points->push_back(Eigen::Vector4d(0.0, 0.0, 0.25, 1.0));
            //
            end_effector_link.link_points->push_back(Eigen::Vector4d(0.0, 0.0, 0.275, 1.0));
            end_effector_link.link_points->push_back(Eigen::Vector4d(0.0, 0.0, 0.3, 1.0));
            end_effector_link.link_points->push_back(Eigen::Vector4d(0.0, 0.0, 0.325, 1.0));
            end_effector_link.link_points->push_back(Eigen::Vector4d(0.0, 0.0, 0.35, 1.0));
            //
            end_effector_link.link_points->push_back(Eigen::Vector4d(0.0, 0.025, 0.275, 1.0));
            end_effector_link.link_points->push_back(Eigen::Vector4d(0.0, 0.025, 0.3, 1.0));
            end_effector_link.link_points->push_back(Eigen::Vector4d(0.0, 0.025, 0.325, 1.0));
            end_effector_link.link_points->push_back(Eigen::Vector4d(0.0, 0.025, 0.35, 1.0));
            //
            end_effector_link.link_points->push_back(Eigen::Vector4d(0.0, 0.05, 0.275, 1.0));
            end_effector_link.link_points->push_back(Eigen::Vector4d(0.0, 0.05, 0.3, 1.0));
            end_effector_link.link_points->push_back(Eigen::Vector4d(0.0, 0.05, 0.325, 1.0));
            end_effector_link.link_points->push_back(Eigen::Vector4d(0.0, 0.05, 0.35, 1.0));
            //
            end_effector_link.link_points->push_back(Eigen::Vector4d(0.0, 0.075, 0.275, 1.0));
            end_effector_link.link_points->push_back(Eigen::Vector4d(0.0, 0.075, 0.3, 1.0));
            end_effector_link.link_points->push_back(Eigen::Vector4d(0.0, 0.075, 0.325, 1.0));
            end_effector_link.link_points->push_back(Eigen::Vector4d(0.0, 0.075, 0.35, 1.0));
            //
            end_effector_link.link_points->push_back(Eigen::Vector4d(0.025, 0.0, 0.275, 1.0));
            end_effector_link.link_points->push_back(Eigen::Vector4d(0.025, 0.0, 0.3, 1.0));
            end_effector_link.link_points->push_back(Eigen::Vector4d(0.025, 0.0, 0.325, 1.0));
            end_effector_link.link_points->push_back(Eigen::Vector4d(0.025, 0.0, 0.35, 1.0));
            //
            end_effector_link.link_points->push_back(Eigen::Vector4d(0.05, 0.0, 0.275, 1.0));
            end_effector_link.link_points->push_back(Eigen::Vector4d(0.05, 0.0, 0.3, 1.0));
            end_effector_link.link_points->push_back(Eigen::Vector4d(0.05, 0.0, 0.325, 1.0));
            end_effector_link.link_points->push_back(Eigen::Vector4d(0.05, 0.0, 0.35, 1.0));
            //
            end_effector_link.link_points->push_back(Eigen::Vector4d(0.075, 0.0, 0.275, 1.0));
            end_effector_link.link_points->push_back(Eigen::Vector4d(0.075, 0.0, 0.3, 1.0));
            end_effector_link.link_points->push_back(Eigen::Vector4d(0.075, 0.0, 0.325, 1.0));
            end_effector_link.link_points->push_back(Eigen::Vector4d(0.075, 0.0, 0.35, 1.0));
        }
        else if (environment_id == "baxter_blocked_test_env")
        {
            // Peg
            end_effector_link.link_points->push_back(Eigen::Vector4d(0.0, 0.0, 0.175, 1.0));
            end_effector_link.link_points->push_back(Eigen::Vector4d(0.0, 0.0, 0.2, 1.0));
            end_effector_link.link_points->push_back(Eigen::Vector4d(0.0, 0.0, 0.225, 1.0));
            end_effector_link.link_points->push_back(Eigen::Vector4d(0.0, 0.0, 0.24, 1.0));
        }
        else
        {
            // Peg
            end_effector_link.link_points->push_back(Eigen::Vector4d(0.0, 0.0, 0.175, 1.0));
            end_effector_link.link_points->push_back(Eigen::Vector4d(0.0, 0.0, 0.2, 1.0));
            end_effector_link.link_points->push_back(Eigen::Vector4d(0.0, 0.0, 0.225, 1.0));
            end_effector_link.link_points->push_back(Eigen::Vector4d(0.0, 0.0, 0.25, 1.0));
        }
    }

    typedef uncertainty_planning_core::LinkedActuatorModel IIWA7JointActuatorModel;

    inline simple_robot_models::SimpleLinkedRobot<IIWA7JointActuatorModel> GetRobot(const Eigen::Affine3d& base_transform, const simple_robot_models::LINKED_ROBOT_CONFIG& joint_config, const std::vector<double>& joint_uncertainty_params, const std::vector<double>& joint_distance_weights, const std::string& environment_id)
    {
        UNUSED(environment_id);
        // Make the robot model
        // Make the link models
        simple_robot_models::RobotLink link_0;
        link_0.link_name = "link_0";
        link_0.link_points->push_back(Eigen::Vector4d(0.0, 0.0, 0.0, 1.0));
        link_0.link_points->push_back(Eigen::Vector4d(0.025, 0.0, 0.0, 1.0));
        link_0.link_points->push_back(Eigen::Vector4d(0.0, 0.025, 0.0, 1.0));
        link_0.link_points->push_back(Eigen::Vector4d(0.0, 0.0, 0.025, 1.0));
        link_0.link_points->push_back(Eigen::Vector4d(0.05, 0.0, 0.0, 1.0));
        link_0.link_points->push_back(Eigen::Vector4d(0.0, 0.05, 0.0, 1.0));
        link_0.link_points->push_back(Eigen::Vector4d(0.075, 0.0, 0.0, 1.0));
        simple_robot_models::RobotLink link_1;
        link_1.link_name = "link_1";
        link_1.link_points->push_back(Eigen::Vector4d(0.0, 0.0, 0.0, 1.0));
        link_1.link_points->push_back(Eigen::Vector4d(0.025, 0.0, 0.0, 1.0));
        link_1.link_points->push_back(Eigen::Vector4d(0.0, 0.025, 0.0, 1.0));
        link_1.link_points->push_back(Eigen::Vector4d(0.0, 0.0, 0.025, 1.0));
        link_1.link_points->push_back(Eigen::Vector4d(0.05, 0.0, 0.0, 1.0));
        link_1.link_points->push_back(Eigen::Vector4d(0.0, 0.05, 0.0, 1.0));
        link_1.link_points->push_back(Eigen::Vector4d(0.075, 0.0, 0.0, 1.0));
        simple_robot_models::RobotLink link_2;
        link_2.link_name = "link_2";
        link_2.link_points->push_back(Eigen::Vector4d(0.0, 0.0, 0.0, 1.0));
        link_2.link_points->push_back(Eigen::Vector4d(0.025, 0.0, 0.0, 1.0));
        link_2.link_points->push_back(Eigen::Vector4d(0.0, 0.025, 0.0, 1.0));
        link_2.link_points->push_back(Eigen::Vector4d(0.0, 0.0, 0.025, 1.0));
        link_2.link_points->push_back(Eigen::Vector4d(0.05, 0.0, 0.0, 1.0));
        link_2.link_points->push_back(Eigen::Vector4d(0.0, 0.05, 0.0, 1.0));
        link_2.link_points->push_back(Eigen::Vector4d(0.075, 0.0, 0.0, 1.0));
        simple_robot_models::RobotLink link_3;
        link_3.link_name = "link_3";
        link_3.link_points->push_back(Eigen::Vector4d(0.0, 0.0, 0.0, 1.0));
        link_3.link_points->push_back(Eigen::Vector4d(0.025, 0.0, 0.0, 1.0));
        link_3.link_points->push_back(Eigen::Vector4d(0.0, 0.025, 0.0, 1.0));
        link_3.link_points->push_back(Eigen::Vector4d(0.0, 0.0, 0.025, 1.0));
        link_3.link_points->push_back(Eigen::Vector4d(0.05, 0.0, 0.0, 1.0));
        link_3.link_points->push_back(Eigen::Vector4d(0.0, 0.05, 0.0, 1.0));
        link_3.link_points->push_back(Eigen::Vector4d(0.075, 0.0, 0.0, 1.0));
        simple_robot_models::RobotLink link_4;
        link_4.link_name = "link_4";
        link_4.link_points->push_back(Eigen::Vector4d(0.0, 0.0, 0.0, 1.0));
        link_4.link_points->push_back(Eigen::Vector4d(0.025, 0.0, 0.0, 1.0));
        link_4.link_points->push_back(Eigen::Vector4d(0.0, 0.025, 0.0, 1.0));
        link_4.link_points->push_back(Eigen::Vector4d(0.0, 0.0, 0.025, 1.0));
        link_4.link_points->push_back(Eigen::Vector4d(0.05, 0.0, 0.0, 1.0));
        link_4.link_points->push_back(Eigen::Vector4d(0.0, 0.05, 0.0, 1.0));
        link_4.link_points->push_back(Eigen::Vector4d(0.075, 0.0, 0.0, 1.0));
        simple_robot_models::RobotLink link_5;
        link_5.link_name = "link_5";
        link_5.link_points->push_back(Eigen::Vector4d(0.0, 0.0, 0.0, 1.0));
        link_5.link_points->push_back(Eigen::Vector4d(0.025, 0.0, 0.0, 1.0));
        link_5.link_points->push_back(Eigen::Vector4d(0.0, 0.025, 0.0, 1.0));
        link_5.link_points->push_back(Eigen::Vector4d(0.0, 0.0, 0.025, 1.0));
        link_5.link_points->push_back(Eigen::Vector4d(0.05, 0.0, 0.0, 1.0));
        link_5.link_points->push_back(Eigen::Vector4d(0.0, 0.05, 0.0, 1.0));
        link_5.link_points->push_back(Eigen::Vector4d(0.075, 0.0, 0.0, 1.0));
        simple_robot_models::RobotLink link_6;
        link_6.link_name = "link_6";
        link_6.link_points->push_back(Eigen::Vector4d(0.0, 0.0, 0.0, 1.0));
        link_6.link_points->push_back(Eigen::Vector4d(0.025, 0.0, 0.0, 1.0));
        link_6.link_points->push_back(Eigen::Vector4d(0.0, 0.025, 0.0, 1.0));
        link_6.link_points->push_back(Eigen::Vector4d(0.0, 0.0, 0.025, 1.0));
        link_6.link_points->push_back(Eigen::Vector4d(0.05, 0.0, 0.0, 1.0));
        link_6.link_points->push_back(Eigen::Vector4d(0.0, 0.05, 0.0, 1.0));
        link_6.link_points->push_back(Eigen::Vector4d(0.075, 0.0, 0.0, 1.0));
        simple_robot_models::RobotLink link_7;
        link_7.link_name = "link_7";
        link_7.link_points->push_back(Eigen::Vector4d(0.0, 0.0, 0.0, 1.0));
        link_7.link_points->push_back(Eigen::Vector4d(0.025, 0.0, 0.0, 1.0));
        link_7.link_points->push_back(Eigen::Vector4d(0.0, 0.025, 0.0, 1.0));
        link_7.link_points->push_back(Eigen::Vector4d(0.0, 0.0, 0.025, 1.0));
        link_7.link_points->push_back(Eigen::Vector4d(0.05, 0.0, 0.0, 1.0));
        link_7.link_points->push_back(Eigen::Vector4d(0.0, 0.05, 0.0, 1.0));
        link_7.link_points->push_back(Eigen::Vector4d(0.075, 0.0, 0.0, 1.0));
        simple_robot_models::RobotLink link_ee;
        link_ee.link_name = "link_ee";
        link_ee.link_points->push_back(Eigen::Vector4d(0.0, 0.0, 0.0, 1.0));
        link_ee.link_points->push_back(Eigen::Vector4d(0.025, 0.0, 0.0, 1.0));
        link_ee.link_points->push_back(Eigen::Vector4d(0.0, 0.025, 0.0, 1.0));
        link_ee.link_points->push_back(Eigen::Vector4d(0.0, 0.0, 0.025, 1.0));
        link_ee.link_points->push_back(Eigen::Vector4d(0.05, 0.0, 0.0, 1.0));
        link_ee.link_points->push_back(Eigen::Vector4d(0.0, 0.05, 0.0, 1.0));
        link_ee.link_points->push_back(Eigen::Vector4d(0.075, 0.0, 0.0, 1.0));
        // Make the joints
        const std::vector<double> joint_max_velocities = GetMaxVelocities();
        const double joint_1_velocity = joint_max_velocities[0];
        const double joint_2_velocity = joint_max_velocities[1];
        const double joint_3_velocity = joint_max_velocities[2];
        const double joint_4_velocity = joint_max_velocities[3];
        const double joint_5_velocity = joint_max_velocities[4];
        const double joint_6_velocity = joint_max_velocities[5];
        const double joint_7_velocity = joint_max_velocities[6];
        const double joint_1_noise = joint_uncertainty_params[0];
        const double joint_2_noise = joint_uncertainty_params[1];
        const double joint_3_noise = joint_uncertainty_params[2];
        const double joint_4_noise = joint_uncertainty_params[3];
        const double joint_5_noise = joint_uncertainty_params[4];
        const double joint_6_noise = joint_uncertainty_params[5];
        const double joint_7_noise = joint_uncertainty_params[6];
        // Make the reference configuration
        const SLC reference_configuration = GetReferenceConfiguration();
        // Joint 1
        simple_robot_models::RobotJoint<IIWA7JointActuatorModel> joint_1;
        joint_1.name = "iiwa_joint_1";
        joint_1.parent_link_index = 0;
        joint_1.child_link_index = 1;
        joint_1.joint_axis = Eigen::Vector3d::UnitZ();
        joint_1.joint_transform = Eigen::Translation3d(0.0, 0.0, 0.15) * EigenHelpers::QuaternionFromUrdfRPY(0.0, 0.0, 0.0);
        joint_1.joint_model = reference_configuration[0];
        const IIWA7JointActuatorModel iiwa_joint_1_model(std::abs(joint_1_noise), 0.5);
        simple_robot_models::LINKED_ROBOT_CONFIG iiwa_joint_1_config = joint_config;
        iiwa_joint_1_config.velocity_limit = joint_1_velocity;
        joint_1.joint_controller = simple_robot_models::JointControllerGroup<IIWA7JointActuatorModel>(iiwa_joint_1_config, iiwa_joint_1_model);
        // Joint 2
        simple_robot_models::RobotJoint<IIWA7JointActuatorModel> joint_2;
        joint_2.name = "iiwa_joint_2";
        joint_2.parent_link_index = 1;
        joint_2.child_link_index = 2;
        joint_2.joint_axis = Eigen::Vector3d::UnitZ();
        joint_2.joint_transform = Eigen::Translation3d(0.0, 0.0, 0.19) * EigenHelpers::QuaternionFromUrdfRPY(M_PI_2, 0.0, M_PI);
        joint_2.joint_model = reference_configuration[1];
        const IIWA7JointActuatorModel iiwa_joint_2_model(std::abs(joint_2_noise), 0.5);
        simple_robot_models::LINKED_ROBOT_CONFIG iiwa_joint_2_config = joint_config;
        iiwa_joint_2_config.velocity_limit = joint_2_velocity;
        joint_2.joint_controller = simple_robot_models::JointControllerGroup<IIWA7JointActuatorModel>(iiwa_joint_2_config, iiwa_joint_2_model);
        // Joint 3
        simple_robot_models::RobotJoint<IIWA7JointActuatorModel> joint_3;
        joint_3.name = "iiwa_joint_3";
        joint_3.parent_link_index = 2;
        joint_3.child_link_index = 3;
        joint_3.joint_axis = Eigen::Vector3d::UnitZ();
        joint_3.joint_transform = Eigen::Translation3d(0.0, 0.21, 0.0) * EigenHelpers::QuaternionFromUrdfRPY(M_PI_2, 0.0, M_PI);
        joint_3.joint_model = reference_configuration[2];
        const IIWA7JointActuatorModel iiwa_joint_3_model(std::abs(joint_3_noise), 0.5);
        simple_robot_models::LINKED_ROBOT_CONFIG iiwa_joint_3_config = joint_config;
        iiwa_joint_3_config.velocity_limit = joint_3_velocity;
        joint_3.joint_controller = simple_robot_models::JointControllerGroup<IIWA7JointActuatorModel>(iiwa_joint_3_config, iiwa_joint_3_model);
        // Joint 4
        simple_robot_models::RobotJoint<IIWA7JointActuatorModel> joint_4;
        joint_4.name = "iiwa_joint_4";
        joint_4.parent_link_index = 3;
        joint_4.child_link_index = 4;
        joint_4.joint_axis = Eigen::Vector3d::UnitZ();
        joint_4.joint_transform = Eigen::Translation3d(0.0, 0.0, 0.19) * EigenHelpers::QuaternionFromUrdfRPY(M_PI_2, 0.0, 0.0);
        joint_4.joint_model = reference_configuration[3];
        const IIWA7JointActuatorModel iiwa_joint_4_model(std::abs(joint_4_noise), 0.5);
        simple_robot_models::LINKED_ROBOT_CONFIG iiwa_joint_4_config = joint_config;
        iiwa_joint_4_config.velocity_limit = joint_4_velocity;
        joint_4.joint_controller = simple_robot_models::JointControllerGroup<IIWA7JointActuatorModel>(iiwa_joint_4_config, iiwa_joint_4_model);
        // Joint 5
        simple_robot_models::RobotJoint<IIWA7JointActuatorModel> joint_5;
        joint_5.name = "iiwa_joint_5";
        joint_5.parent_link_index = 4;
        joint_5.child_link_index = 5;
        joint_5.joint_axis = Eigen::Vector3d::UnitZ();
        joint_5.joint_transform = Eigen::Translation3d(0.0, 0.21, 0.0) * EigenHelpers::QuaternionFromUrdfRPY(-M_PI_2, M_PI, 0.0);
        joint_5.joint_model = reference_configuration[4];
        const IIWA7JointActuatorModel iiwa_joint_5_model(std::abs(joint_5_noise), 0.5);
        simple_robot_models::LINKED_ROBOT_CONFIG iiwa_joint_5_config = joint_config;
        iiwa_joint_5_config.velocity_limit = joint_5_velocity;
        joint_5.joint_controller = simple_robot_models::JointControllerGroup<IIWA7JointActuatorModel>(iiwa_joint_5_config, iiwa_joint_5_model);
        // Joint 6
        simple_robot_models::RobotJoint<IIWA7JointActuatorModel> joint_6;
        joint_6.name = "iiwa_joint_6";
        joint_6.parent_link_index = 5;
        joint_6.child_link_index = 6;
        joint_6.joint_axis = Eigen::Vector3d::UnitZ();
        joint_6.joint_transform = Eigen::Translation3d(0.0, 0.06070, 0.19) * EigenHelpers::QuaternionFromUrdfRPY(M_PI_2, 0.0, 0.0);
        joint_6.joint_model = reference_configuration[5];
        const IIWA7JointActuatorModel iiwa_joint_6_model(std::abs(joint_6_noise), 0.5);
        simple_robot_models::LINKED_ROBOT_CONFIG iiwa_joint_6_config = joint_config;
        iiwa_joint_6_config.velocity_limit = joint_6_velocity;
        joint_6.joint_controller = simple_robot_models::JointControllerGroup<IIWA7JointActuatorModel>(iiwa_joint_6_config, iiwa_joint_6_model);
        // Joint 7
        simple_robot_models::RobotJoint<IIWA7JointActuatorModel> joint_7;
        joint_7.name = "iiwa_joint_7";
        joint_7.parent_link_index = 6;
        joint_7.child_link_index = 7;
        joint_7.joint_axis = Eigen::Vector3d::UnitZ();
        joint_7.joint_transform = Eigen::Translation3d(0.0, 0.081, 0.06070) * EigenHelpers::QuaternionFromUrdfRPY(-M_PI_2, M_PI, 0.0);
        joint_7.joint_model = reference_configuration[6];
        const IIWA7JointActuatorModel iiwa_joint_7_model(std::abs(joint_7_noise), 0.5);
        simple_robot_models::LINKED_ROBOT_CONFIG iiwa_joint_7_config = joint_config;
        iiwa_joint_7_config.velocity_limit = joint_7_velocity;
        joint_7.joint_controller = simple_robot_models::JointControllerGroup<IIWA7JointActuatorModel>(iiwa_joint_7_config, iiwa_joint_7_model);
        // Joint EE
        simple_robot_models::RobotJoint<IIWA7JointActuatorModel> joint_ee;
        joint_ee.name = "iiwa_joint_ee";
        joint_ee.parent_link_index = 7;
        joint_ee.child_link_index = 8;
        joint_ee.joint_axis = Eigen::Vector3d::UnitZ();
        joint_ee.joint_transform = Eigen::Translation3d(0.0, 0.0, 0.045) * EigenHelpers::QuaternionFromUrdfRPY(0.0, -M_PI_2, 0.0);
        joint_ee.joint_model = SJM(std::make_pair(0.0, 0.0), 0.0, SJM::FIXED);
        // We don't need an uncertainty model for a fixed joint
        joint_ee.joint_controller = simple_robot_models::JointControllerGroup<IIWA7JointActuatorModel>(joint_config);
        // Put together
        const std::vector<simple_robot_models::RobotLink> links = {link_0, link_1, link_2, link_3, link_4, link_5, link_6, link_7, link_ee};
        const std::vector<std::pair<size_t, size_t>> allowed_self_collisions = {std::pair<size_t, size_t>(0, 1), std::pair<size_t, size_t>(1, 2), std::pair<size_t, size_t>(2, 3), std::pair<size_t, size_t>(3, 4), std::pair<size_t, size_t>(4, 5), std::pair<size_t, size_t>(5, 6), std::pair<size_t, size_t>(6, 7), std::pair<size_t, size_t>(7, 8)};
        const std::vector<simple_robot_models::RobotJoint<IIWA7JointActuatorModel>> joints = {joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, joint_7, joint_ee};
        const simple_robot_models::SimpleLinkedRobot<IIWA7JointActuatorModel> robot(base_transform, links, joints, allowed_self_collisions, reference_configuration, joint_distance_weights);
        return robot;
    }

    inline uncertainty_planning_core::LinkedSamplerPtr GetSampler()
    {
        // Make the sampler
        const SLC reference_configuration = GetReferenceConfiguration();
        return uncertainty_planning_core::LinkedSamplerPtr(new simple_samplers::SimpleLinkedBaseSampler<uncertainty_planning_core::PRNG>(reference_configuration));
    }

    inline uncertainty_planning_core::LinkedSimulatorPtr GetSimulator(const config_common::TASK_CONFIG_PARAMS& options, const int32_t debug_level)
    {
        const simulator_environment_builder::EnvironmentComponents environment_components = simulator_environment_builder::BuildCompleteEnvironment(options.environment_id, options.environment_resolution);
        const fast_kinematic_simulator::SolverParameters solver_params = fast_kinematic_simulator::GetDefaultSolverParameters();
        return fast_kinematic_simulator::MakeLinkedSimulator(environment_components.GetEnvironment(), environment_components.GetEnvironmentSDF(), environment_components.GetSurfaceNormalsGrid(), solver_params, options.simulation_controller_frequency, debug_level);
    }
}

#endif // IIWA7_LINKED_COMMON_CONFIG_HPP
