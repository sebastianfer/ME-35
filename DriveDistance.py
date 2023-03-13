import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionClient

from irobot_create_msgs.action import DriveDistance

class DriveDistanceActionClient(Node):
    def __init__(self):
        super().__init__('drive_distance_action_client')
        self._action_client = ActionClient(self, DriveDistance, '/drive_distance')

    def send_goal(self, distance = 0.5, max_translation_speed = 0.3):
        goal_msg = DriveDistance.Goal()
        goal_msg.distance = distance
        goal_msg.max_translation_speed = max_translation_speed

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
    drive_distance_action_client = DriveDistanceActionClient()
    try:
       rclpy.spin(drive_distance_action_client)
    except KeyboardInterrupt:
       print('\nAction concluded')
    finally:
       print('Done')
       rclpy.shutdown()
        
        
if __name__ == '__main__':
     main()
