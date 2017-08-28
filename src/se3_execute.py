#!/usr/bin/python

#####################################################
#                                                   #
#   Copyright (c) 2015, Calder Phillips-Grafflin    #
#                                                   #
#   SE(3) Execute Shim                              #
#                                                   #
#####################################################

import rospy
import std_srvs.srv
import uncertainty_planning_examples.srv
import actionlib
import thruster_robot_controllers.msg
import thruster_robot_examples.srv


class ExecuteServer(object):

    def __init__(self, service_path, command_action, abort_service, teleport_service):
        self.command_action_client = actionlib.SimpleActionClient(command_action, thruster_robot_controllers.msg.GoToPoseTargetAction)
        self.command_action_client.wait_for_server()
        rospy.loginfo("Connected to action server")
        self.abort_client = rospy.ServiceProxy(abort_service, std_srvs.srv.Empty)
        self.abort_client.wait_for_service()
        rospy.loginfo("Connected to abort server")
        self.teleport_client = rospy.ServiceProxy(teleport_service, thruster_robot_examples.srv.Teleport)
        self.teleport_client.wait_for_service()
        rospy.loginfo("Connected to teleport server")
        self.server = rospy.Service(service_path, uncertainty_planning_examples.srv.Simple6dofRobotMove, self.service_handler)
        rospy.loginfo("...ready")
        spin_rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            spin_rate.sleep()
        rospy.loginfo("Shutting down")

    def service_handler(self, request):
        rospy.loginfo("Received execution service call")
        if request.mode == uncertainty_planning_examples.srv.Simple6dofRobotMoveRequest.RESET:
            rospy.loginfo("Resetting to " + str(request.start))
            self.command_stop()
            rospy.sleep(2.5)
            print "first time tele"
            self.command_teleport(request.start)
            trajectory = [request.start]
            # rospy.sleep(2.5)
            # self.command_stop()
            # rospy.sleep(2.5)
            # print "second time tele"
            # self.command_teleport(request.start)
        elif request.mode == uncertainty_planning_examples.srv.Simple6dofRobotMoveRequest.EXECUTE:
            rospy.loginfo("Executing to " + str(request.target))
            robot_target = request.target
            control_mode = request.control_mode
            file_path = request.file_path
            contact_info = request.contact_info
            trajectory = self.command_to_target(robot_target, request.time_limit, control_mode, file_path, contact_info)
        elif request.mode == uncertainty_planning_examples.srv.Simple6dofRobotMoveRequest.EXECUTE_FROM_START:
            rospy.loginfo("First, resetting to " + str(request.start))
            self.command_stop()
            rospy.sleep(2.5)
            self.command_teleport(request.start)
            rospy.loginfo("Executing to " + str(request.target))
            robot_target = request.target
            trajectory = self.command_to_target(robot_target, request.time_limit)
        else:
            rospy.logerr("Invalid mode command")
            trajectory = []
        rospy.loginfo("Assembling response")
        response = uncertainty_planning_examples.srv.Simple6dofRobotMoveResponse()
        response.trajectory = trajectory
        rospy.loginfo("Response with " + str(len(response.trajectory)) + " states")
        # rospy.loginfo("Reached " + str(response.trajectory[-1]))
        return response

    def command_teleport(self, target_pose):
        req = thruster_robot_examples.srv.TeleportRequest()
        req.target_pose = target_pose
        self.teleport_client.call(req)

    def command_to_target(self, target_pose, time_limit, control_mode = [], file_path = "", contact_info = []):
        goal = thruster_robot_controllers.msg.GoToPoseTargetGoal()
        goal.max_execution_time = time_limit
        goal.target_pose = target_pose
        goal.control_mode = control_mode
        goal.file_path = file_path
        goal.contact_info = contact_info
        self.command_action_client.send_goal(goal)
        self.command_action_client.wait_for_result()
        result = self.command_action_client.get_result()
        return result.trajectory

    def command_stop(self):
        self.command_action_client.cancel_all_goals()
        req = std_srvs.srv.EmptyRequest()
        self.abort_client.call(req)

if __name__ == '__main__':
    rospy.init_node("simple6dof_execute_server")
    rospy.loginfo("Starting...")
    ExecuteServer("simple_6dof_robot_move", "vehicle_bus/go_to_target_pose", "vehicle_bus/shim_abort", "vehicle_bus/bus/teleport")
    # ExecuteServer("simple_6dof_robot_move", "vehicle_bus/go_to_target_pose", "vehicle_bus/target_pose/abort", "vehicle_bus/bus/teleport")