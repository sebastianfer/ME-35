import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionClient

from irobot_create_msgs.action import RotateAngle

class RotateAngleActionClient(Node):
    def __init__(self):
        super().__init__('rotate_angle_action_client')
        
        # Positive angles will make the robot move left, negative angles will make the robot move right
        angle = float(input('Angle of Rotation (radians): '))
        self._action_client = ActionClient(self, RotateAngle, '/rotate_angle')
        self.send_goal(angle)

    def send_goal(self, angle, max_rotation_speed = 1.9):
        print('Ready for Action')

        goal_msg = RotateAngle.Goal()
        goal_msg.angle = angle
        goal_msg.max_rotation_speed = max_rotation_speed

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')


def main(args=None):
    rclpy.init(args=args)
    rotate_angle_action_client = RotateAngleActionClient()
    try:
       rclpy.spin(rotate_angle_action_client)
    except KeyboardInterrupt:
       print('\nAction concluded')
    finally:
       print('Done')
       rclpy.shutdown()
        
        
if __name__ == '__main__':
     main()
