###############################################################################
# ME35
# Navigate to Position
#
# This file creates a single action client for one robot that utilizes the 
# odometry tool of the Create3 and calls the NavigateToPosition action to make 
# the robot move to a sepcific destination
#
# Authors: Sebastian Fernandez, Alex Savic
# Date: 04/17/23
# Updated: 04/19/23
###############################################################################

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
import time
from irobot_create_msgs.action import NavigateToPosition


class NavigateToPositionActionClient(Node):
    def __init__(self):
        super().__init__('navigate_to_position_action_client')
        self._action_client = ActionClient(self, NavigateToPosition, 'navigate_to_position')
        print('Creating Action Client')

        self.current_x = 0
        self.current_y = 0
        self.goal_x = 0
        self.goal_y = 0

        self.current_rotation_angle = 0
        self.goal_rotation_angle = 0

        self.odom_reset = False
        self.goal_achieved = False

    def send_goal(self, goal_x, goal_y, goal_rotation_z, goal_rotation_w):
        print('Sending Goal')
        goal_msg = NavigateToPosition.Goal()
        goal_msg.achieve_goal_heading = True
        goal_msg.goal_pose.pose.position.x = goal_x
        goal_msg.goal_pose.pose.position.y = goal_y
        goal_msg.goal_pose.pose.orientation.z = goal_rotation_z
        goal_msg.goal_pose.pose.orientation.w = goal_rotation_w

        print("Waiting for server")
        self._action_client.wait_for_server()
        print('Navigate to Position')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        print("Send goal async")
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):

        # Store the result of the future as a new variable named 'goal_handle'
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.pose))
        # Shut down rclpy
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback nav state: {0}'.format(feedback.navigate_state))
        self.get_logger().info('Received feedback remaining angle: {0}'.format(feedback.remaining_angle_travel))
        self.get_logger().info('Received feedback remaining distance: {0}'.format(feedback.remaining_travel_distance))

def main(args=None):
    rclpy.init(args=args)
    action_client = NavigateToPositionActionClient()
    action_client.send_goal(float(0.70),float(0),float(0.707), float(0.707))  # this values will make the robot move 0.7 m and then rotate 90 degrees to the left
    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        rclpy.shutdown()
if __name__ == '__main__':
    main()
