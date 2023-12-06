#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from geometry_msgs.msg import Twist

import rclpy
import sys

from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile


class BasicNavigator(Node):
    def __init__(self):
        super().__init__(node_name='basic_navigator')
        self.initial_pose = Pose()
        self.goal_handle = None
        self.result_future = None
        self.feedback = None
        self.status = None

        amcl_pose_qos = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=1)
        
        self.motor_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.initial_pose_received = False
        self.nav_through_poses_client = ActionClient(self,
                                                     NavigateThroughPoses,
                                                     'navigate_through_poses')
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.model_pose_sub = self.create_subscription(PoseWithCovarianceStamped,
                                                       'amcl_pose',
                                                       self._amclPoseCallback,
                                                       amcl_pose_qos)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped,
                                                      'initialpose',
                                                      10)
        self.lift_pub = self.create_publisher(Bool, 'lift', 10)
        self.pallet_center_service_client = self.create_client(SetBool, 'activate_pallet_center')

    def setInitialPose(self, initial_pose):
        self.initial_pose_received = False
        self.initial_pose = initial_pose
        self._setInitialPose()

    def goThroughPoses(self, poses):
        # Sends a `NavToPose` action request and waits for completion
        self.debug("Waiting for 'NavigateToPose' action server")
        while not self.nav_through_poses_client.wait_for_server(timeout_sec=1.0):
            self.info("'NavigateToPose' action server not available, waiting...")

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = poses

        self.info('Navigating with ' + str(len(poses)) + ' goals.' + '...')
        send_goal_future = self.nav_through_poses_client.send_goal_async(goal_msg,
                                                                         self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Goal with ' + str(len(poses)) + ' poses was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def goToPose(self, pose):
        # Sends a `NavToPose` action request and waits for completion
        self.debug("Waiting for 'NavigateToPose' action server")
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.info("'NavigateToPose' action server not available, waiting...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.info('Navigating to goal: ' + str(pose.pose.position.x) + ' ' +
                      str(pose.pose.position.y) + '...')
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg,
                                                                   self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Goal to ' + str(pose.pose.position.x) + ' ' +
                           str(pose.pose.position.y) + ' was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def cancelNav(self):
        self.info('Canceling current goal.')
        if self.result_future:
            future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future)
        return

    def isNavComplete(self):
        if not self.result_future:
            # task was cancelled or completed
            return True
        rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.10)
        if self.result_future.result():
            self.status = self.result_future.result().status
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                self.info('Goal with failed with status code: {0}'.format(self.status))
                return True
        else:
            # Timed out, still processing, not complete yet
            return False

        self.info('Goal succeeded!')
        return True

    def getFeedback(self):
        return self.feedback

    def getResult(self):
        return self.status

    def waitUntilNav2Active(self):
        self._waitForNodeToActivate('amcl')
        self._waitForInitialPose()
        self._waitForNodeToActivate('bt_navigator')
        self.info('Nav2 is ready for use!')
        return

    def _waitForNodeToActivate(self, node_name):
        # Waits for the node within the tester namespace to become active
        self.debug('Waiting for ' + node_name + ' to become active..')
        node_service = node_name + '/get_state'
        state_client = self.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            self.info(node_service + ' service not available, waiting...')

        req = GetState.Request()
        state = 'unknown'
        while (state != 'active'):
            self.debug('Getting ' + node_name + ' state...')
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                state = future.result().current_state.label
                self.debug('Result of get_state: %s' % state)
            time.sleep(2)
        return

    def _waitForInitialPose(self):
        while not self.initial_pose_received:
            self.info('Setting initial pose')
            self._setInitialPose()
            self.info('Waiting for amcl_pose to be received')
            rclpy.spin_once(self, timeout_sec=1)
        return

    def _amclPoseCallback(self, msg):
        self.initial_pose_received = True
        return

    def _feedbackCallback(self, msg):
        self.feedback = msg.feedback
        return

    def _setInitialPose(self):
        msg = PoseWithCovarianceStamped()
        msg.pose.pose = self.initial_pose
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        self.info('Publishing Initial Pose')
        self.initial_pose_pub.publish(msg)
        return

    def info(self, msg):
        self.get_logger().info(msg)
        return

    def warn(self, msg):
        self.get_logger().warn(msg)
        return

    def error(self, msg):
        self.get_logger().error(msg)
        return

    def debug(self, msg):
        self.get_logger().debug(msg)
        return
    
    def lift_up(self):
        lift_msg = Bool()
        lift_msg.data = True
        self.navigator.lift_pub.publish(lift_msg)

    def lift_down(self):
        lift_msg = Bool()
        lift_msg.data = False
        self.navigator.lift_pub.publish(lift_msg)    
        
    def activate_pallet_center(self, activate=True):
        while not self.pallet_center_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for PalletCenter service...')
        request = SetBool.Request()
        request.data = activate
        future = self.pallet_center_service_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
    
    def move_backward(self, duration):
        backward_msg = Twist()
        backward_msg.linear.x = -0.1  # 후진 속도 설정
        self.motor_publisher.publish(backward_msg)
        time.sleep(duration)
        self.stop_robot()

    def stop_robot(self):
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.motor_publisher.publish(stop_msg)

def main(argv=sys.argv[1:]):
    rclpy.init()
    navigator = BasicNavigator()

    # Set our demo's initial pose
    initial_pose = Pose()
    initial_pose.position.x = 0.0
    initial_pose.position.y = 0.0
    initial_pose.orientation.z = 0.0
    initial_pose.orientation.w = 1.0
    #navigator.setInitialPose(initial_pose)

    # Wait for navigation to fully activate
    navigator.waitUntilNav2Active()

    # Go to our demos initial pose
    initial_goal_pose = PoseStamped()
    initial_goal_pose.header.frame_id = 'map'
    initial_goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_goal_pose.pose = initial_pose

    # Go to our demos first goal pose
    goal_pose_1 = PoseStamped()
    goal_pose_1.header.frame_id = 'map'
    goal_pose_1.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose_1.pose.position.x = 3.1343 #3.0326
    goal_pose_1.pose.position.y = -0.3119 #-1.0428
    goal_pose_1.pose.orientation.z = -0.71512
    goal_pose_1.pose.orientation.w = 0.69901

    goal_pose_2 = PoseStamped()
    goal_pose_2.header.frame_id = 'map'
    goal_pose_2.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose_2.pose.position.x = 3.1698
    goal_pose_2.pose.position.y = -2.2366
    goal_pose_2.pose.orientation.z = -0.71206
    goal_pose_2.pose.orientation.w = 0.70212


    i = 0

    while True:
        user_input = input("Enter the command & up down: ")
        
        if user_input == "1":
            navigator.goToPose(goal_pose_1)
            while not navigator.isNavComplete():
                pass  # 계속 기다립니다.

        elif user_input == "2":
            navigator.goToPose(goal_pose_2)
            while not navigator.isNavComplete():
                pass  # 계속 기다립니다.

        elif user_input == "3":
            navigator.goToPose(initial_goal_pose)
            while not navigator.isNavComplete():
                pass

        elif user_input == "start":
            navigator.goToPose(initial_goal_pose)
            while not navigator.isNavComplete():
                pass 
            
            navigator.goThroughPoses([goal_pose_1, goal_pose_2])
            while not navigator.isNavComplete():
                pass

            response = navigator.activate_pallet_center(True)
            if not response.success:
                navigator.lift_up()
                time.sleep(3)
                navigator.move_backward(3)
                navigator.goToPose(initial_goal_pose)
                
        elif user_input == "":
            navigator.lift_up()
            time.sleep(3)

            navigator.goToPose(goal_pose_2)
            while not navigator.isNavComplete():
                pass 

            navigator.goToPose(initial_goal_pose)
            while not navigator.isNavComplete():
                pass
        
        elif user_input == "return":
            navigator.lift_down()
            time.sleep(3)

            navigator.goToPose(initial_goal_pose)
            while not navigator.isNavComplete():
                pass


        elif user_input == "up":
            lift_msg = Bool()
            lift_msg.data = True
            navigator.lift_pub.publish(lift_msg)
            print("/lift : True")

        elif user_input == "down":
            lift_msg = Bool()
            lift_msg.data = False
            navigator.lift_pub.publish(lift_msg)
            print("/lift : False")

        else:
            print("Invalid input, try again.")
    # while not navigator.isNavComplete():
    #     ################################################
    #     #
    #     # Implement some code here for your application!
    #     #
    #     ################################################

    #     # Do something with the feedback
    #     i = i + 1
    #     feedback = navigator.getFeedback()
    #     #if feedback and i % 5 == 0:
    #         #print('Estimated time of arrival: ' + '{0:.0f}'.format(
    #               #Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  
    #             #+ ' seconds.')

    #         # Some navigation timeout to demo cancellation
    #         #if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
    #            # navigator.cancelNav()

    # # Do something depending on the return code
    # result = navigator.getResult()
    # if result == GoalStatus.STATUS_SUCCEEDED:
    #     print('Goal succeeded!')
    #     lift_msg = Bool()
    #     lift_msg.data = True
    #     navigator.lift_pub.publish(lift_msg)
        
    #     navigator.goToPose(initial_goal_pose)
    #     while not navigator.isNavComplete():
    #         pass

    #     lift_msg.data = False
    #     navigator.lift_pub.publish(lift_msg)

    # elif result == GoalStatus.STATUS_CANCELED:
    #     print('Goal was canceled!')
    # elif result == GoalStatus.STATUS_ABORTED:
    #     print('Goal failed!')
    # else:
    #     print('Goal has an invalid return status!')

    #exit(0)
