###############################################################################
# ME35
# Final Project 
# main.py
#
# This file creates a single action client for one robot that utilizes the 
# odometry tool of the Create3 and calls the NavigateToPosition action to make 
# the robot move to a sepcific destination
#
# Authors: Sebastian Fernandez, Alex Savic
# Date: 04/17/23
# Updated: 
###############################################################################
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
import time
from irobot_create_msgs.action import NavigateToPosition
from irobot_create_msgs.srv import ResetPose
from irobot_create_msgs.action import Dock
from irobot_create_msgs.action import Undock


class MovementActionClient(Node):
    def __init__(self):
        super().__init__('navigate_to_position_action_client')
        self.navigate = ActionClient(self, NavigateToPosition, 'navigate_to_position')
        self.dock = ActionClient(self, Dock, 'dock')
        self.undock = ActionClient(self, Undock, 'undock')
        print('Creating Action Client')

        self.current_x = 0
        self.current_y = 0
        self.goal_x = 0
        self.goal_y = 0

        self.current_rotation_angle = 0
        self.goal_rotation_angle = 0
    
    def send_navigate(self, goal_x, goal_y, goal_rotation_z, goal_rotation_w):
        print('Sending Goal to Navigate to Position')
        goal_msg = NavigateToPosition.Goal()
        goal_msg.achieve_goal_heading = True
        goal_msg.goal_pose.pose.position.x = goal_x
        goal_msg.goal_pose.pose.position.y = goal_y
        goal_msg.goal_pose.pose.orientation.z = goal_rotation_z
        goal_msg.goal_pose.pose.orientation.w = goal_rotation_w

        print("Waiting for server")
        self.navigate.wait_for_server()
        self._send_goal_future = self.navigate.send_goal_async(goal_msg, feedback_callback = self.navigate_feedback_callback)
        self._send_goal_future.add_done_callback(self.navigate_response_callback)

    def navigate_response_callback(self, future):
        # Store the result of the future as a new variable named 'goal_handle'
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.navigate_result_callback)
    
    def navigate_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.pose))
        self.send_dock()

    def navigate_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback nav state: {0}'.format(feedback.navigate_state))
        self.get_logger().info('Received feedback remaining angle: {0}'.format(feedback.remaining_angle_travel))
        self.get_logger().info('Received feedback remaining distance: {0}'.format(feedback.remaining_travel_distance))

    def send_dock(self):
        print('Sending Goal to Dock')
        goal_msg = Dock.Goal()
        self.dock.wait_for_server()
        self._send_goal_future = self.dock.send_goal_async(goal_msg, feedback_callback = self.dock_feedback_callback)
        self._send_goal_future.add_done_callback(self.dock_response_callback)

    def dock_response_callback(self, future):
        # Store the result of the future as a new variable named 'goal_handle'
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.dock_result_callback)
    
    def dock_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.is_docked))
        time.sleep(1)
        self.send_undock()

    def dock_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback nav state: {0}'.format(feedback.sees_dock))

    def send_undock(self):
        print('Sending Goal to Undock')
        goal_msg = Undock.Goal()
        self.undock.wait_for_server()
        self._send_goal_future = self.undock.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.undock_response_callback)

    def undock_response_callback(self, future):
        # Store the result of the future as a new variable named 'goal_handle'
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.undock_result_callback)
    
    def undock_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))
        rclpy.shutdown()

    
def main(args=None):
    rclpy.init(args=args)
    action_client = MovementActionClient()

    action_client.send_navigate(float(0.70),float(0),float(0.707), float(0.707))
    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        rclpy.shutdown()
if __name__ == '__main__':
    main()
