###############################################################################
# ME35
# Final Project 
# SmoothieBar.py
#
# This file creates a single action client for one robot that utilizes the 
# odometry tool of the Create3 and calls the NavigateToPosition action to make 
# the robot move to a sepcific destination
#
# Authors: Sebastian Fernandez, Alex Savic
# Date: 04/17/23
# Updated: 04/20/23
###############################################################################
import rclpy
import RPi.GPIO as GPIO
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

URL = 'https://api.airtable.com/v0/appxFSx1vodDWljqe/Transport1?api_key=keyKf7fEzdNw4S7qi'

coordinates = None

# this is information for pushing data to the airtable
AIRTABLE_API_KEY = 'keyKf7fEzdNw4S7qi'
AIRTABLE_BASE_ID = 'appxFSx1vodDWljqe'
TRANSPORT1 = 'Transport1'
TRANSPORT2 = 'Transport2'

headers = {
    "Authorization": f"Bearer {AIRTABLE_API_KEY}",
    "Content-Type": "application/json"
}

######################################

## These are for uploading to the Airtable ## 
# Transport 1
cupPickUpID1 = 'recPdXmglpJuWSWEM'
dispenserDock1ID1 = 'recXUQhRaZQ0FdAy7'
dispenserDock2ID1 = 'recQrj9xx2XFTXros'
processedPickupID1 = 'recliJaPTW7ZtazNd'
blender1ID = 'recsOKXrgKTC6wwHZ'
pickup1ID1 = 'reczZPqJ1MUudKV8t'

cupEndpoint1 = f'https://api.airtable.com/v0/{AIRTABLE_BASE_ID}/TRANSPORT1/{cupPickUpID1}'
dispenserDock1Endpoint1 = f'https://api.airtable.com/v0/{AIRTABLE_BASE_ID}/TRANSPORT1/{dispenserDock1ID1}'
dispenserDock2Endpoint1 = f'https://api.airtable.com/v0/{AIRTABLE_BASE_ID}/TRANSPORT1/{dispenserDock2ID1}'
processedPickupEndpoint1 = f'https://api.airtable.com/v0/{AIRTABLE_BASE_ID}/TRANSPORT1/{processedPickupID1}'
blender1Endpoint1 = f'https://api.airtable.com/v0/{AIRTABLE_BASE_ID}/TRANSPORT1/{blender1ID}'
pickup1Endpoint1 = f'https://api.airtable.com/v0/{AIRTABLE_BASE_ID}/TRANSPORT1/{pickup1ID1}'

###################################
class cup_check:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)

        #define the pin that goes to the circuit
        self.pin_to_circuit = 4

    def rc_time (self):
        count = 0

        #Output on the pin for
        GPIO.setup(self.pin_to_circuit, GPIO.OUT)
        GPIO.output(self.pin_to_circuit, GPIO.LOW)
        time.sleep(0.1)

        #Change the pin back to input
        GPIO.setup(self.pin_to_circuit, GPIO.IN)

        #Count until the pin goes high
        while (GPIO.input(self.pin_to_circuit) == GPIO.LOW):
            count += 1
            
            if count >= 20000:
                break

        return count

    def main (self):
        #Catch when script is interrupted, cleanup correctly
        try:
            # Main loop
            while True:
                                             
                force = self.rc_time()
                                                               
                if force < 20000:                 
                    print("occupied")
                    return 1
                    break
                else:
                    print("empty")
                    return 0
                    break
             
        except KeyboardInterrupt:
            pass
        finally:
            print("clean")
            GPIO.cleanup()

class cup_publish:
    def __init__ (self):
        # this is information for pushing data to the airtable
        self.AIRTABLE_API_KEY = 'keyKf7fEzdNw4S7qi'
        self.AIRTABLE_BASE_ID = 'appxFSx1vodDWljqe'
        self.TRANSPORT1 = 'Transport1'
        self.TRANSPORT2 = 'Transport2'

        self.headers = {
            "Authorization": f"Bearer {self.AIRTABLE_API_KEY}",
            "Content-Type": "application/json"
        }
        ## These are for uploading to the Airtable ## 
        # Transport 1
        self.cupPickUpID1 = 'recPdXmglpJuWSWEM'
        self.pickup1ID1 = 'reczZPqJ1MUudKV8t'
        
        self.cupEndpoint1 = f'https://api.airtable.com/v0/{self.AIRTABLE_BASE_ID}/TRANSPORT1/{self.cupPickUpID1}'
        self.pickup1Endpoint1 = f'https://api.airtable.com/v0/{self.AIRTABLE_BASE_ID}/TRANSPORT1/{self.pickup1ID1}'
        self.cup_check = cup_check()
    
    def publish_pickup(self):
        while True:
            status = cup_check.main()
            
            if status == 1:
                break
        data = {
                "fields": {
                    "Task Finished": 1
                }
            }
        r = requests.patch(self.cupEndpoint1, json=data, headers=self.headers)
            
    def publish_delivery(self):
        while True:
            status = cup_check.main()
            
            if status == 0:
                break
        data = {
                "fields": {
                    "Task Finished": 1
                }
            }
        r = requests.patch(self.pickup1Endpoint1, json=data, headers=self.headers)
            
            
test = cup_publish()


class MovementActionClient(Node):
    def __init__(self):
        super().__init__('navigate_to_position_action_client')
        self.navigate = ActionClient(self, NavigateToPosition, 'navigate_to_position')
        self.dock = ActionClient(self, Dock, 'dock')
        self.undock = ActionClient(self, Undock, 'undock')
        self.timer = self.create_timer(1, self.timer_callback)
        print('Created Action Client')

        self.station = 0
        self.totalStations = 5
    
    def timer_callback(self):
        ready_array = getReady()
        finished_array = getFinish()

        next_ready_value = ready_array[(self.station + 1) % self.totalStations]
        finished_value = finished_array[self.station]
        print("Current station: " + str(self.station))
        print("Next station ready value: " + str(next_ready_value))
        print("Current station finished value: " + str(finished_value))
        
        if (self.station == 0):
            test.publish_pickup()
        elif (self.station == self.totalStations - 1):
            test.publish_delivery()

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

    #def refresh_airtable_callback(self):

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
        if (self.station != 4):
            self.send_dock()
        else:
            self.station = (self.station + 1) % self.totalStations

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
            data = {
                "fields": {
                    "Ready for Robot": 1,
                    "Robot Docked": 0,
                    "Task Finished": 0
                }
            }
            r = requests.patch(cupEndpoint1, json=data, headers=headers)
        elif self.station == 1:
            data = {
                "fields": {
                    "Ready for Robot": 1,
                    "Robot Docked": 0,
                    "Task Finished": 0
                }
            }
            r = requests.patch(dispenserDock1Endpoint1, json=data, headers=headers)
        elif self.station == 2:
            data = {
                "fields": {
                    "Ready for Robot": 1,
                    "Robot Docked": 0,
                    "Task Finished": 0
                }
            }
            r = requests.patch(dispenserDock2Endpoint1, json=data, headers=headers)
        elif self.station == 3:
            data = {
                "fields": {
                    "Ready for Robot": 1,
                    "Robot Docked": 0,
                    "Task Finished": 0
                }
            }
            r = requests.patch(processedPickupEndpoint1, json=data, headers=headers)
        elif self.station == 4:
            data = {
                "fields": {
                    "Ready for Robot": 1,
                    "Robot Docked": 0,
                    "Task Finished": 0
                }
            }
            r = requests.patch(blender1Endpoint1, json=data, headers=headers)
        else:
            data = {
                "fields": {
                    "Ready for Robot": 1,
                    "Robot Docked": 0,
                    "Task Finished": 0
                }
            }
            r = requests.patch(pickup1Endpoint1, json=data, headers=headers)


def getReady():
    response = requests.get(url = URL, params = {})
    datatask = response.json()
    # retrieves boolean for if station is ready or not
    Rcup = datatask['records'][0]['fields']['Ready for Robot']
    Rdispenser2 = datatask['records'][1]['fields']['Ready for Robot']
    Rdispenser1 = datatask['records'][2]['fields']['Ready for Robot']
    Rblender1 = datatask['records'][4]['fields']['Ready for Robot']
    Rpickup1 = datatask['records'][5]['fields']['Ready for Robot']
    Rprocessed = datatask['records'][3]['fields']['Ready for Robot']

    ready_array = [Rcup, Rdispenser1, Rdispenser2, Rprocessed, Rblender1, Rpickup1]
    
    return ready_array

def getDocked():
    response = requests.get(url = URL, params = {})
    datatask = response.json()
    # retrieves boolean for if robot is docked or not
    Dcup = datatask['records'][0]['fields']['Robot Docked']
    Ddispenser2 = datatask['records'][1]['fields']['Robot Docked']
    Ddispenser1 = datatask['records'][2]['fields']['Robot Docked']
    Dblender1 = datatask['records'][4]['fields']['Robot Docked']
    Dpickup1 = datatask['records'][5]['fields']['Robot Docked']
    Dprocessed = datatask['records'][3]['fields']['Robot Docked']

    docked_array = [Dcup, Ddispenser1, Ddispenser2, Dprocessed, Dblender1, Dpickup1]

    
    return docked_array

def getFinish():
    # retrieves boolean for if robot is finished with task
    response = requests.get(url = URL, params = {})
    datatask = response.json()

    Fcup = datatask['records'][0]['fields']['Task Finished']
    Fdispenser2 = datatask['records'][1]['fields']['Task Finished']
    Fdispenser1 = datatask['records'][2]['fields']['Task Finished']
    Fblender1 = datatask['records'][4]['fields']['Task Finished']
    Fpickup1 = datatask['records'][5]['fields']['Task Finished']
    Fprocessed = datatask['records'][3]['fields']['Task Finished']

    finished_array = [Fcup, Fdispenser1, Fdispenser2, Fprocessed, Fblender1, Fpickup1]

    return finished_array

def getCoordinates(): #retrieves coordiates for every task
    response = requests.get(url = URL, params = {})
    datatask = response.json()

    Cupx = float(datatask['records'][0]['fields']['X Coordinate'])
    Cupy = float(datatask['records'][0]['fields']['Y Coordinate'])  
    Cupz = float(datatask['records'][0]['fields']['Z Rotation']) 
    Cupw = float(datatask['records'][0]['fields']['W Rotation'])
    
    Disp2x = float(datatask['records'][1]['fields']['X Coordinate'])
    Disp2y = float(datatask['records'][1]['fields']['Y Coordinate']) 
    Disp2z = float(datatask['records'][1]['fields']['Z Rotation'])
    Disp2w = float(datatask['records'][1]['fields']['W Rotation'])
    
    Disp1x = float(datatask['records'][2]['fields']['X Coordinate'])
    Disp1y = float(datatask['records'][2]['fields']['Y Coordinate']) 
    Disp1z = float(datatask['records'][2]['fields']['Z Rotation'])
    Disp1w = float(datatask['records'][2]['fields']['W Rotation'])
    
    Prox = float(datatask['records'][3]['fields']['X Coordinate'])
    Proy = float(datatask['records'][3]['fields']['Y Coordinate']) 
    Proz = float(datatask['records'][3]['fields']['Z Rotation'])
    Prow = float(datatask['records'][3]['fields']['W Rotation'])
    
    B1x = float(datatask['records'][4]['fields']['X Coordinate'])
    B1y = float(datatask['records'][4]['fields']['Y Coordinate']) 
    B1z = float(datatask['records'][4]['fields']['Z Rotation'])
    B1w = float(datatask['records'][4]['fields']['W Rotation'])
    
    P1x = float(datatask['records'][5]['fields']['X Coordinate'])
    P1y = float(datatask['records'][5]['fields']['Y Coordinate']) 
    P1z = float(datatask['records'][5]['fields']['Z Rotation'])
    P1w = float(datatask['records'][5]['fields']['W Rotation'])

    return [[Cupx, Cupy, Cupz, Cupw], [Disp1x, Disp1y, Disp1z, Disp1w], [Disp2x, Disp2y, Disp2z, Disp2w], [Prox, Proy, Proz, Prow], [B1x, B1y, B1z, B1w], [P1x, P1y, P1z, P1w]]

    
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
