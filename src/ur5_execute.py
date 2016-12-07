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
import baxter_robot_interface.msg
import baxter_robot_interface.srv


class ExecuteServer(object):

    def __init__(self, service_path, command_action, abort_service, teleport_service):
        self.command_action_client = actionlib.SimpleActionClient(command_action, baxter_robot_interface.msg.GoToJointTargetAction)
        self.command_action_client.wait_for_server()
        rospy.loginfo("Connected to action server")
        self.abort_client = rospy.ServiceProxy(abort_service, std_srvs.srv.Empty)
        self.abort_client.wait_for_service()
        rospy.loginfo("Connected to abort server")
        self.teleport_client = rospy.ServiceProxy(teleport_service, gazebo_msgs.srv.SetModelConfiguration)
        self.teleport_client.wait_for_service()
        rospy.loginfo("Connected to teleport server")
        self.server = rospy.Service(service_path, uncertainty_planning_examples.srv.SimpleLinkedRobotMove, self.service_handler)
        rospy.loginfo("...ready")
        spin_rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            spin_rate.sleep()
        rospy.loginfo("Shutting down")

    def service_handler(self, request):
        rospy.loginfo("Received execution service call")
        if request.mode == uncertainty_planning_examples.srv.Simple6dofRobotMoveRequest.RESET:
            rospy.loginfo("Resetting to " + str(zip(request.joint_name, request.start_position)))
            self.command_stop()
            rospy.sleep(2.5)
            self.command_teleport(request.joint_name, request.start_position)
            result = control_msgs.msg.JointTrajectoryControllerState()
            result.joint_names = request.joint_name
            result.actual.positions = request.start_position
            trajectory = [result]
        elif request.mode == uncertainty_planning_examples.srv.Simple6dofRobotMoveRequest.EXECUTE:
            rospy.loginfo("Executing to " + str(zip(request.joint_name, request.target_position)))
            trajectory = self.command_to_target(request.joint_name, request.target_position, request.max_execution_time)
        elif request.mode == uncertainty_planning_examples.srv.Simple6dofRobotMoveRequest.EXECUTE_FROM_START:
            rospy.loginfo("First, resetting to " + str(zip(request.joint_name, request.start_position)))
            self.command_stop()
            rospy.sleep(2.5)
            self.command_teleport(request.joint_name, request.start_position)
            rospy.loginfo("Executing to " + str(zip(request.joint_name, request.target_position)))
            trajectory = self.command_to_target(request.joint_name, request.target_position, request.max_execution_time)
        else:
            rospy.logerr("Invalid mode command")
            trajectory = []
        rospy.loginfo("Assembling response")
        response = uncertainty_planning_examples.srv.SimpleLinkedRobotMoveResponse()
        response.trajectory = trajectory
        rospy.loginfo("Response with " + str(len(response.trajectory)) + " states")
        return response

    def command_teleport(self, joint_names, target_positions):
        req = gazebo_msgs.srv.SetModelConfigurationRequest()
        req.model_name = "baxter"
        req.urdf_param_name = "robot_description"
        req.joint_names = joint_names
        req.joint_positions = target_positions
        self.teleport_client.call(req)

    def command_to_target(self, joint_names, target_positions, time_limit):
        goal = baxter_robot_interface.msg.GoToJointTargetGoal()
        goal.max_execution_time = time_limit
        goal.target.name = joint_names
        goal.target.position = target_positions
        self.command_action_client.send_goal(goal)
        self.command_action_client.wait_for_result()
        result = self.command_action_client.get_result()
        return result.trajectory

    def command_stop(self):
        req = std_srvs.srv.EmptyRequest()
        self.abort_client.call(req)

if __name__ == '__main__':
    rospy.init_node("simplelinked_execute_server")
    rospy.loginfo("Starting...")
    ExecuteServer("simple_linked_robot_move", "baxter_robot_position_controller/go_to_joint_target", "baxter_robot_position_controller/abort", "gazebo/set_model_configuration")
