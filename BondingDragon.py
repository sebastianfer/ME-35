import sys 
import rclpy
import requests
import json

from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3


class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher')
        print('Creating publisher')
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.wheels = Twist()
        self.linear = Vector3()
        self.angular = Vector3()
        timer_period = 5
        self.time = self.create_timer(timer_period, self.WheelVelocity)
        
    def WheelVelocity(self):
        URL = 'https://api.airtable.com/v0/appOrOyyzywWzK1At/Velocities/?api_key=keyb5LWBSnWfRFG8M'
        r = requests.get(url = URL, params = {})
        data = r.json()    # this returns dictionary
        velocity = data['records'][0]['fields']
        
        for key, value in velocity.items():
           if key == 'X Linear Velocity':
               self.linear.x = float(velocity[key])
           elif key == 'X Angular Velocity':
               self.angular.x = float(velocity[key])
           elif key == 'Y Linear Velocity':
               self.linear.y = float(velocity[key])
           elif key == 'Y Angular Velocity':
               self.angular.y = float(velocity[key])
           elif key == 'Z Linear Velocity':
               self.linear.z = float(velocity[key])
           elif key == 'Z Angular Velocity':
               self.angular.z = float(velocity[key])
        self.wheels.linear = self.linear
        self.wheels.angular = self.angular
        self.vel_publisher.publish(self.wheels)
        
def main(args=None):
    rclpy.init(args=args)
    velocity_publisher = VelocityPublisher()
    print('Ready to run')
    try:
        rclpy.spin(velocity_publisher)
    except KeyboardInterrupt:
        print('\nCaught keyboard interrupt')
    finally:
        print("Done")
        velocity_publisher.destroy_node()
        print('shutting down')
        rclpy.shutdown()


if __name__ == '__main__':
    main()