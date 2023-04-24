###############################################################################
# ME35
# Final Project 
# SmoothieBar.py
#
# This file creates a single action client for one robot that utilizes the 
# odometry tool of the Create3 and calls the NavigateToPosition action to make 
# the robot move to a sepcific destination
#
# Authors: Sebastian Fernandez, Alex Savic, Casey Lam
# Date: 04/17/23
# Updated: 04/24/23
###############################################################################
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
import time
import requests
import json
from irobot_create_msgs.action import NavigateToPosition
from irobot_create_msgs.srv import ResetPose
from irobot_create_msgs.action import Dock
from irobot_create_msgs.action import Undock

URL = 'https://api.airtable.com/v0/appxFSx1vodDWljqe/Transport_Order?api_key=keyKf7fEzdNw4S7qi'
coordinates = None

class MovementActionClient(Node):
    def __init__(self):
        super().__init__('navigate_to_position_action_client')
        self.navigate = ActionClient(self, NavigateToPosition, 'navigate_to_position')
        self.dock = ActionClient(self, Dock, 'dock')
        self.undock = ActionClient(self, Undock, 'undock')
        self.timer = self.create_timer(0.5, self.timer_callback)
        print('Created Action Client')

        self.station = 0
        self.totalStations = 8
    
    def timer_callback(self):
        ready_array = getReady()
        finished_array = getFinish()

        next_ready_value = ready_array[(self.station + 1) % self.totalStations]
        finished_value = finished_array[self.station]
        print("Current station: " + str(self.station))
        print("Next station ready value: " + str(next_ready_value))
        print("Current station finished value: " + str(finished_value))
        
        if ((finished_value == 1) and (next_ready_value == 1)):
            self.send_undock()

    
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
        self.station = (self.station + 1) % self.totalStations

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
        global coordinates
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))
        self.update_airtable()
        next_coordinates = coordinates[(self.station + 1) % self.totalStations]
        self.send_navigate(next_coordinates[0], next_coordinates[1], next_coordinates[2], next_coordinates[3])

    def update_airtable(self):
        response = requests.get(url = URL, params = {})
        datatask = response.json()
        print("Airtable values being updated")
        if self.station == 0:
            datatask['records'][0]['fields']['Ready for Robot'] = 1
            datatask['records'][0]['fields']['Robot Docked'] = 0
            datatask['records'][0]['fields']['Task Finished'] = 0
        elif self.station == 1:
            datatask['records'][2]['fields']['Ready for Robot'] = 1
            datatask['records'][2]['fields']['Robot Docked'] = 0
            datatask['records'][2]['fields']['Task Finished'] = 0
        elif self.station == 2:
            datatask['records'][1]['fields']['Ready for Robot'] = 1
            datatask['records'][1]['fields']['Robot Docked'] = 0
            datatask['records'][1]['fields']['Task Finished'] = 0
        elif self.station == 3:
            datatask['records'][5]['fields']['Ready for Robot'] = 1
            datatask['records'][5]['fields']['Robot Docked'] = 0
            datatask['records'][5]['fields']['Task Finished'] = 0
        elif self.station == 4:
            datatask['records'][6]['fields']['Ready for Robot'] = 1
            datatask['records'][6]['fields']['Robot Docked'] = 0
            datatask['records'][6]['fields']['Task Finished'] = 0
        elif self.station == 5:
            datatask['records'][3]['fields']['Ready for Robot'] = 1
            datatask['records'][3]['fields']['Robot Docked'] = 0
            datatask['records'][3]['fields']['Task Finished'] = 0
        elif self.station == 6:
            datatask['records'][7]['fields']['Ready for Robot'] = 1
            datatask['records'][7]['fields']['Robot Docked'] = 0
            datatask['records'][7]['fields']['Task Finished'] = 0
        else:   
            datatask['records'][4]['fields']['Ready for Robot'] = 1
            datatask['records'][4]['fields']['Robot Docked'] = 0
            datatask['records'][4]['fields']['Task Finished'] = 0

def getReady():
    response = requests.get(url = URL, params = {})
    datatask = response.json()
    # retrieves boolean for if station is ready or not
    Rcup = datatask['records'][0]['fields']['Ready for Robot']
    Rsmall = datatask['records'][1]['fields']['Ready for Robot']
    Rice = datatask['records'][2]['fields']['Ready for Robot']
    Rblender2 = datatask['records'][3]['fields']['Ready for Robot']
    Rpickup2 = datatask['records'][4]['fields']['Ready for Robot']
    Rprocessed = datatask['records'][5]['fields']['Ready for Robot']
    Rblender1 = datatask['records'][6]['fields']['Ready for Robot']
    Rpickup1 = datatask['records'][7]['fields']['Ready for Robot']

    ready_array = [Rcup, Rice, Rsmall, Rprocessed, Rblender1, Rblender2, Rpickup1, Rpickup2]

    
    return ready_array

def getDocked():
    response = requests.get(url = URL, params = {})
    datatask = response.json()
    # retrieves boolean for if robot is docked or not
    Dcup = datatask['records'][0]['fields']['Robot Docked']
    Dsmall = datatask['records'][1]['fields']['Robot Docked']
    Dice = datatask['records'][2]['fields']['Robot Docked']
    Dblender2 = datatask['records'][3]['fields']['Robot Docked']
    Dpickup2 = datatask['records'][4]['fields']['Robot Docked']
    Dprocessed = datatask['records'][5]['fields']['Robot Docked']
    Dblender1 = datatask['records'][6]['fields']['Robot Docked']
    Dpickup1 = datatask['records'][7]['fields']['Robot Docked']
    
    docked_array = [Dcup, Dice, Dsmall, Dprocessed, Dblender1, Dblender2, Dpickup1, Dpickup2]

    
    return docked_array

def getFinish():
    # retrieves boolean for if robot is finished with task
    response = requests.get(url = URL, params = {})
    datatask = response.json()

    Fcup = datatask['records'][0]['fields']['Task Finished']
    Fsmall = datatask['records'][1]['fields']['Task Finished']
    Fice = datatask['records'][2]['fields']['Task Finished']
    Fblender2 = datatask['records'][3]['fields']['Task Finished']
    Fpickup2 = datatask['records'][4]['fields']['Task Finished']
    Fprocessed = datatask['records'][5]['fields']['Task Finished']
    Fblender1 = datatask['records'][6]['fields']['Task Finished']
    Fpickup1 = datatask['records'][7]['fields']['Task Finished']
    
    finished_array = [Fcup, Fice, Fsmall, Fprocessed, Fblender1, Fblender2, Fpickup1, Fpickup2]

    return finished_array

def getCoordinates(): #retrieves coordiates for every task
    response = requests.get(url = URL, params = {})
    datatask = response.json()

    Cupx = float(datatask['records'][0]['fields']['X Coordinate'])
    Cupy = float(datatask['records'][0]['fields']['Y Coordinate'])  
    Cupz = float(datatask['records'][0]['fields']['Z Rotation']) 
    Cupw = float(datatask['records'][0]['fields']['W Rotation'])
    
    Smallx = float(datatask['records'][1]['fields']['X Coordinate'])
    Smally = float(datatask['records'][1]['fields']['Y Coordinate']) 
    Smallz = float(datatask['records'][1]['fields']['Z Rotation'])
    Smallw = float(datatask['records'][1]['fields']['W Rotation'])
    
    Icex = float(datatask['records'][2]['fields']['X Coordinate'])
    Icey = float(datatask['records'][2]['fields']['Y Coordinate']) 
    Icez = float(datatask['records'][2]['fields']['Z Rotation'])
    Icew = float(datatask['records'][2]['fields']['W Rotation'])
    
    B2x = float(datatask['records'][3]['fields']['X Coordinate'])
    B2y = float(datatask['records'][3]['fields']['Y Coordinate']) 
    B2z = float(datatask['records'][3]['fields']['Z Rotation'])
    B2w = float(datatask['records'][3]['fields']['W Rotation'])
    
    P2x = float(datatask['records'][4]['fields']['X Coordinate'])
    P2y = float(datatask['records'][4]['fields']['Y Coordinate']) 
    P2z = float(datatask['records'][4]['fields']['Z Rotation'])
    P2w = float(datatask['records'][4]['fields']['W Rotation'])
    
    Prox = float(datatask['records'][5]['fields']['X Coordinate'])
    Proy = float(datatask['records'][5]['fields']['Y Coordinate']) 
    Proz = float(datatask['records'][5]['fields']['Z Rotation'])
    Prow = float(datatask['records'][5]['fields']['W Rotation'])
    
    B1x = float(datatask['records'][6]['fields']['X Coordinate'])
    B1y = float(datatask['records'][6]['fields']['Y Coordinate']) 
    B1z = float(datatask['records'][6]['fields']['Z Rotation'])
    B1w = float(datatask['records'][6]['fields']['W Rotation'])
    
    P1x = float(datatask['records'][7]['fields']['X Coordinate'])
    P1y = float(datatask['records'][7]['fields']['Y Coordinate']) 
    P1z = float(datatask['records'][7]['fields']['Z Rotation'])
    P1w = float(datatask['records'][7]['fields']['W Rotation'])

    return [[Cupx, Cupy, Cupz, Cupw], [Icex, Icey, Icez, Icew], [Smallx, Smally, Smallz, Smallw], [Prox, Proy, Proz, Prow], [B1x, B1y, B1z, B1w], [B2x, B2y, B2z, B2w], [P1x, P1y, P1z, P1w], [P2x, P2y, P2z, P2w]]

    
def main(args=None):
    rclpy.init(args=args)
    action_client = MovementActionClient()
    global coordinates
    coordinates = getCoordinates()

    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()
