# Import Libraries for camera
from keras.models import load_model  # TensorFlow is required for Keras to work
import cv2  # Install opencv-python
import numpy as np
from picamera2 import Picamera2
from libcamera import controls
import time

# Import ROS 2 and iRobot libraries
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionClient
from irobot_create_msgs.action import RotateAngle
from irobot_create_msgs.action import DriveDistance
from irobot_create_msgs.msg import LedColor
from irobot_create_msgs.msg import LightringLeds
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

picam2 = Picamera2()
picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous}) # sets auto focus mode
picam2.start() # activates camera
time.sleep(0.5)

# Disable scientific notation for clarity
np.set_printoptions(suppress=True)
# Load the model
model = load_model('keras_model.h5', compile=False)
# Load the labels
class_names = open('labels.txt', 'r').readlines()

# Outputs are Confidence Score and Class
def TakePictures():
    global confidence_score
    img_name = "/home/tuftsrobot/Robotics/ros2_files/image.jpg"
    img = picam2.capture_file(img_name)
    image = cv2.imread("/home/tuftsrobot/Robotics/ros2_files/image.jpg") #taking image and overriding it

    # Resize the raw image into (224-height,224-width) pixels
    image = cv2.resize(image, (224, 224), interpolation=cv2.INTER_AREA)

    # Make the image a numpy array and reshape it to the models input shape.
    image = np.asarray(image, dtype=np.float32).reshape(1, 224, 224, 3)
    # Normalize the image array
    image = (image / 127.5) - 1
    # Predicts the model
    prediction = model.predict(image)
    index = np.argmax(prediction)
    class_name = class_names[index]
    confidence_score = prediction[0][index]
    # Print prediction and confidence score
    print('Class:', class_name[2:], end='')
    print('Confidence Score:', str(np.round(confidence_score * 100))[:-2], '%')
    return [class_name, confidence_score]

class RotateAngleActionClient(Node):
    def __init__(self, class_name):
        super().__init__('rotate_angle_action_client')
        self._action_client = ActionClient(self, RotateAngle, '/rotate_angle')

    def send_goal(self, angle, max_rotation_speed = 1.9):
        goal_msg = RotateAngle.Goal()
        goal_msg.angle = angle
        goal_msg.max_rotation_speed = max_rotation_speed

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('RotateAngle rejected :(')
            return

        self.get_logger().info('RotateAngle accepted :)')

class Dance(Node):
    def __init__(self):
        super().__init__('dance')
        self.dance_publisher = self.create_publisher(Twist, '/cmd_vel', 10)   # publish to /cmd_vel
        self.wheels = Twist()
        self.linear = Vector3()
        self.angular = Vector3()
        timer_period = 0.5
        self.time = self.create_timer(timer_period, self.WheelVelocity)
        
    def WheelVelocity(self):
        self.linear.x = float(0.0)
        self.linear.y = float(0.0)
        self.linear.z = float(0.0)
        self.angular.x = float(0.0)
        self.angular.y = float(0.0)
        self.angular.z = float(-2.5)

        self.wheels.linear = self.linear
        self.wheels.angular = self.angular
        self.dance_publisher.publish(self.wheels)

class DriveDistanceActionClient(Node):
    def __init__(self):
        super().__init__('drive_distance_action_client')
        self._action_client = ActionClient(self, DriveDistance, '/drive_distance')

    def send_goal(self, distance = 0.0762, max_translation_speed = 0.3):
        goal_msg = DriveDistance.Goal()
        goal_msg.distance = distance
        goal_msg.max_translation_speed = max_translation_speed

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('DriveDistance rejected :(')
            return

        self.get_logger().info('DriveDistance accepted :)')

class ColorPalette():
    def __init__(self):
        self.red = LedColor(red=255, green=0, blue=0)
        self.green = LedColor(red=0, green=255, blue=0)
        self.blue = LedColor(red=0, green=0, blue=255)
        self.yellow = LedColor(red=255, green=255, blue=0)
        self.pink = LedColor(red=255, green=0, blue=255)
        self.cyan = LedColor(red=0, green=255, blue=255)
        self.purple = LedColor(red=127, green=0, blue=255)
        self.white = LedColor(red=255, green=255, blue=255)
        self.grey = LedColor(red=189, green=189, blue=189)
        self.tufts_blue = LedColor(red=98, green=166, blue=10)
        self.tufts_brown = LedColor(red=94, green=75, blue=60)


class LEDPublisher(Node):
    def __init__(self):
        super().__init__('led_publisher')
        self.cp = ColorPalette()
        self.lights_publisher = self.create_publisher(LightringLeds, '/cmd_lightring', 10)
        timer_period = 0.6
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.lightring = LightringLeds()
        self.lightring.override_system = True

    def timer_callback(self):
        color_1 = [self.cp.red, self.cp.green, self.cp.blue, self.cp.purple, self.cp.yellow, self.cp.grey]
        color_2 = [self.cp.grey, self.cp.red, self.cp.green, self.cp.blue, self.cp.purple, self.cp.yellow]
        color_3 = [self.cp.yellow, self.cp.grey, self.cp.red, self.cp.green, self.cp.blue, self.cp.purple]
        color_4 = [self.cp.purple, self.cp.yellow, self.cp.grey, self.cp.red, self.cp.green, self.cp.blue]
        color_5 = [self.cp.blue, self.cp.purple, self.cp.yellow, self.cp.grey, self.cp.red, self.cp.green]
        color_6 = [self.cp.green, self.cp.blue, self.cp.purple, self.cp.yellow, self.cp.grey, self.cp.red]
        color_cycle = (color_1,color_2,color_3,color_4,color_5,color_6)
        current_time = self.get_clock().now()

        for i in range(5):
            self.lightring.leds = color_cycle[i]
            self.lightring.header.stamp = current_time.to_msg()
            self.lights_publisher.publish(self.lightring)
            time.sleep(0.1)

    def reset(self):
        self.lightring.override_system = False
        white = [self.cp.white, self.cp.white, self.cp.white,
                 self.cp.white, self.cp.white, self.cp.white]
        self.lightring.leds = white
        self.lights_publisher.publish(self.lightring)


def main(args=None):
    angle = 0       # Initialize angle
    rclpy.init(args=args)
    try:
        while True:
            [class_name, confidence_score] = TakePictures()
            # Positive angles will make the robot rotate to the left
            # Negative angles will make the robot rotate to the right
            if class_name == class_names[0]:    # Elephant
                angle = -1.57
            elif class_name == class_names[1]:  # Tractor
                angle = -1.57
            elif class_name == class_names[2]:   # Kiwi
                angle = -1.57
            elif class_name == class_names[3]:    # Bear
                angle = -1.57
            elif class_name == class_names[4]:    # Mug
                angle = -3.14
            elif class_name == class_names[5]:    # Mario
                angle = -1.57
            elif class_name == class_names[6]:    # Cube
                angle = -1.57
                
            if confidence_score < 0.90:
                drive_distance_action_client = DriveDistanceActionClient()
                drive_distance_action_client.send_goal()
                rclpy.spin_once(drive_distance_action_client)

            # See anything except elephant and bear    
            elif (confidence_score >= 0.90) and (class_name != class_names[0]) and (class_name != class_names[4]):
                rotate_angle_action_client = RotateAngleActionClient(class_name)
                rotate_angle_action_client.send_goal(float(angle))
                rclpy.spin_once(rotate_angle_action_client)

            # See elephant
            elif (confidence_score > 0.9996 and class_name == class_names[0]):
                rotate_angle_action_client = RotateAngleActionClient(class_name)
                rotate_angle_action_client.send_goal(float(angle))
                rclpy.spin_once(rotate_angle_action_client)

            # Sees elephant but it's not very accurate
            elif (confidence_score >= 0.90 and confidence_score < 1) and (class_name == class_names[0]):
                drive_distance_action_client = DriveDistanceActionClient()
                drive_distance_action_client.send_goal()
                rclpy.spin_once(drive_distance_action_client)

            # Sees Kiwi
            elif (confidence_score > 0.8 and class_name == class_names[2]):
                print("KIWI")
                rotate_angle_action_client = RotateAngleActionClient(class_name)
                rotate_angle_action_client.send_goal(float(angle))
                rclpy.spin_once(rotate_angle_action_client)
            
            # Sees Mug
            elif (confidence_score >= 0.90 and class_name == class_names[4]):
                rotate_angle_action_client = RotateAngleActionClient(class_name)
                rotate_angle_action_client.send_goal(float(angle))
                rclpy.spin_once(rotate_angle_action_client)               
                raise Exception 

    except Exception:
        print('Dance')
        led_publisher = LEDPublisher()
        rclpy.spin_once(led_publisher) 
        dance = Dance()
        for i in range(15):
            rclpy.spin_once(dance)
        led_publisher.reset()
    except KeyboardInterrupt:
        print('\nAction concluded')
    finally: 
        rclpy.shutdown()

if __name__ == '__main__':
     main()
