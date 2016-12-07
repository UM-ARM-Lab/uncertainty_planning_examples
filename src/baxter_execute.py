#!/usr/bin/python

#####################################################
#                                                   #
#   Copyright (c) 2015, Calder Phillips-Grafflin    #
#                                                   #
#   Baxter Execute Shim                             #
#                                                   #
#####################################################

import rospy
import std_srvs.srv
import uncertainty_planning_examples.srv
import actionlib
import control_msgs.msg
import gazebo_msgs.srv
import baxter_core_msgs.msg
import baxter_robot_interface.msg
import baxter_robot_interface.srv


class ExecuteServer(object):

    def __init__(self, velocity_command_topic, service_path, command_action, position_abort_service, velocity_abort_service, teleport_service):
        self.velocity_command_publisher = rospy.Publisher(velocity_command_topic, baxter_core_msgs.msg.JointCommand, queue_size=1)
        self.command_action_client = actionlib.SimpleActionClient(command_action, baxter_robot_interface.msg.GoToJointTargetAction)
        self.command_action_client.wait_for_server()
        rospy.loginfo("Connected to action server")
        self.position_abort_client = rospy.ServiceProxy(position_abort_service, std_srvs.srv.Empty)
        self.position_abort_client.wait_for_service()
        rospy.loginfo("Connected to position abort server")
        self.velocity_abort_client = rospy.ServiceProxy(velocity_abort_service, std_srvs.srv.Empty)
        self.velocity_abort_client.wait_for_service()
        rospy.loginfo("Connected to velocity abort server")
        if teleport_service != "":
            self.teleport_client = rospy.ServiceProxy(teleport_service, gazebo_msgs.srv.SetModelConfiguration)
            self.teleport_client.wait_for_service()
            rospy.loginfo("Connected to teleport server")
        else:
            self.teleport_client = None
            rospy.logwarn("Teleport service disabled")
        self.server = rospy.Service(service_path, uncertainty_planning_examples.srv.SimpleLinkedRobotMove, self.service_handler)
        rospy.loginfo("...ready")
        spin_rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            spin_rate.sleep()
        rospy.loginfo("Shutting down")

    def service_handler(self, request):
        rospy.loginfo("Received execution service call")
        if request.mode == uncertainty_planning_examples.srv.SimpleLinkedRobotMoveRequest.RESET:
            rospy.loginfo("Resetting to " + str(zip(request.joint_name, request.start_position)))
            self.command_stop()
            rospy.sleep(2.5)
            self.command_teleport(request.joint_name, request.start_position)
            result = control_msgs.msg.JointTrajectoryControllerState()
            result.joint_names = request.joint_name
            result.actual.positions = request.start_position
            trajectory = [result]
        elif request.mode == uncertainty_planning_examples.srv.SimpleLinkedRobotMoveRequest.EXECUTE:
            rospy.loginfo("Executing to " + str(zip(request.joint_name, request.target_position)))
            trajectory = self.command_to_target(request.joint_name, request.target_position, request.expected_result_position, request.max_execution_time, request.execution_shortcut_distance)
        elif request.mode == uncertainty_planning_examples.srv.SimpleLinkedRobotMoveRequest.EXECUTE_FROM_START:
            rospy.loginfo("First, resetting to " + str(zip(request.joint_name, request.start_position)))
            self.command_stop()
            rospy.sleep(2.5)
            self.command_teleport(request.joint_name, request.start_position)
            rospy.loginfo("Executing to " + str(zip(request.joint_name, request.target_position)))
            trajectory = self.command_to_target(request.joint_name, request.target_position, request.expected_result_position, request.max_execution_time, request.execution_shortcut_distance)
        else:
            rospy.logerr("Invalid mode command")
            trajectory = []
        rospy.loginfo("Assembling response")
        response = uncertainty_planning_examples.srv.SimpleLinkedRobotMoveResponse()
        response.trajectory = trajectory
        rospy.loginfo("Response with " + str(len(response.trajectory)) + " states")
        if len(response.trajectory) > 0:
            reached_state = response.trajectory[-1]
            rospy.loginfo("Reached " + str(zip(reached_state.joint_names, reached_state.actual.positions)))
        else:
            rospy.logwarn("Response trajectory is empty")
        return response

    def command_teleport(self, joint_names, target_positions):
        if self.teleport_client is not None:
            names = ["right_s0", "right_s1", "right_e0", "right_e1", "right_w0", "right_w1", "right_w2"]
            for name in names:
                req = gazebo_msgs.srv.SetModelConfigurationRequest()
                req.model_name = "baxter"
                req.urdf_param_name = "robot_description"
                req.joint_names = [name]
                req.joint_positions = [target_positions[joint_names.index(name)]]
                res = self.teleport_client.call(req)
                print(res)
        else:
            rospy.loginfo("Teleport ignored")

    def command_to_target(self, joint_names, target_positions, expected_result_positions, time_limit, execution_shortcut_distance):
        goal = baxter_robot_interface.msg.GoToJointTargetGoal()
        goal.max_execution_time = time_limit
        goal.execution_shortcut_distance = execution_shortcut_distance
        goal.target.actual.positions = expected_result_positions
        goal.target.desired.positions = target_positions
        goal.target.joint_names = joint_names
        self.command_action_client.send_goal(goal)
        self.command_action_client.wait_for_result()
        result = self.command_action_client.get_result()
        return result.trajectory

    def command_stop(self):
        req = std_srvs.srv.EmptyRequest()
        self.position_abort_client.call(req)
        rospy.sleep(0.5)
        req = std_srvs.srv.EmptyRequest()
        self.velocity_abort_client.call(req)
        rospy.sleep(0.5)
        stop_command = baxter_core_msgs.msg.JointCommand()
        stop_command.mode = baxter_core_msgs.msg.JointCommand.VELOCITY_MODE
        stop_command.names = ["right_s0", "right_s1", "right_e0", "right_e1", "right_w0", "right_w1", "right_w2"]
        stop_command.command = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.velocity_command_publisher.publish(stop_command)
        rospy.sleep(0.0)

if __name__ == '__main__':
    rospy.init_node("simplelinked_execute_server")
    rospy.loginfo("Starting...")
    can_teleport = rospy.get_param("~can_teleport", True)
    if can_teleport:
        ExecuteServer("/robot/limb/right/joint_command_velocity_noisy", "simple_linked_robot_move", "baxter_robot_position_controller/go_to_joint_target", "baxter_right_arm_position_controller/abort", "baxter_right_arm_velocity_torque_controller/abort", "gazebo/set_model_configuration")
    else:
        ExecuteServer("/robot/limb/right/joint_command_velocity_noisy", "simple_linked_robot_move", "baxter_robot_position_controller/go_to_joint_target", "baxter_right_arm_position_controller/abort", "baxter_right_arm_velocity_torque_controller/abort", "")
