import carla
from carla_msgs.msg import CarlaWorldInfo
from nav_msgs.msg import Path

from carla_msgs.msg import CarlaStatus
from carla_msgs.msg import CarlaEgoVehicleInfo
from carla_msgs.msg import CarlaEgoVehicleStatus
from carla_msgs.msg import CarlaEgoVehicleControl

import ros_compatibility as roscomp
from ros_compatibility.exceptions import *
from ros_compatibility.node import CompatibleNode
from ros_compatibility.qos import QoSProfile, DurabilityPolicy

import math
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
import time
import csv

class WayPointSubscriber(CompatibleNode):
    def __init__(self):
        super().__init__('waypoint_subscriber')
        self.connect_to_carla()
        fast_qos = QoSProfile(depth=10)
        self.goal_subscriber = self.create_subscription(Path, "/carla/ego_vehicle/waypoints", self.waypoint_visualizer, qos_profile=10)
        self.vehicle_control_publisher = self.create_publisher(CarlaEgoVehicleControl, "/carla/ego_vehicle/vehicle_control_cmd", qos_profile=10)

        self.PREFERRED_SPEED = 20 # what it says
        self.SPEED_THRESHOLD = 0.1 #defines when we get close to desired speed so we drop the
        self.speed = 0

        self.estimated_throttle = 0.0
        self.steer_input = 0.0

        #adding params to display text to image
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        # org - defining lines to display telemetry values on the screen
        org = (30, 30) # this line will be used to show current speed
        self.org2 = (30, 50) # this line will be used for future steering angle
        org3 = (30, 70) # and another line for future telemetry outputs
        org4 = (30, 90) # and another line for future telemetry outputs
        org3 = (30, 110) # and another line for future telemetry outputs
        self.fontScale = 0.5
        # white color
        self.color = (255, 255, 255)
        # Line thickness of 2 px
        self.thickness = 1

        self.hero = None
        for actor in self.world.get_actors():
            if actor.attributes.get('role_name') == 'ego_vehicle':
                self.hero = actor
                break

        #self.image_subscription = self.create_subscription(Image, "/carla/ego_vehicle/rgb_view/image", self.camera_rgb_view, 10)
        #self.image_subscription
        self.br = CvBridge()

        # save vehicle speed to analyze
        '''
        self.csvfile = open('/home/nithish/monocular_object_detection/CAROM/data_analysis/vehicle_speed_analysis/40kph.csv', 'a', newline='')
        self.csvwriter = csv.writer(self.csvfile)
        '''

    def angle_between(self, v1, v2):
        return math.degrees(np.arctan2(v1[1], v1[0]) - np.arctan2(v2[1], v2[0]))
    
    def get_angle(self, car, wp):
        '''
        this function to find direction to selected waypoint
        '''
        vehicle_pos = car.get_transform()
        car_x = vehicle_pos.location.x
        car_y = vehicle_pos.location.y
        wp_x = wp.x
        wp_y = wp.y
        
        # vector to waypoint
        x = (wp_x - car_x)/((wp_y - car_y)**2 + (wp_x - car_x)**2)**0.5
        y = (wp_y - car_y)/((wp_y - car_y)**2 + (wp_x - car_x)**2)**0.5
        
        #car vector
        car_vector = vehicle_pos.get_forward_vector()
        degrees = self.angle_between((x,y),(car_vector.x,car_vector.y))

        return degrees
    
    def maintain_speed(self, s):
        ''' 
        this is a very simple function to maintan desired speed
        s arg is actual current speed
        '''
        if s >= self.PREFERRED_SPEED:
            return 0.25
        elif s < self.PREFERRED_SPEED - self.SPEED_THRESHOLD:
            return 0.5 # think of it as % of "full gas"
        else:
            return 0.4 # tweak this if the car is way over or under preferred speed

    def camera_rgb_view(self, camera_msg):
        self.camera_frame = self.br.imgmsg_to_cv2(camera_msg)
        self.camera_frame = cv2.putText(self.camera_frame, 'Speed: '+str(self.speed)+' kmh', self.org2, 
                        self.font, self.fontScale, self.color, self.thickness, cv2.LINE_AA)
        cv2.imshow('RGB view', self.camera_frame)
        cv2.waitKey(1)

    def waypoint_visualizer(self, msg):
        self.loginfo('Waypoints: {}'.format(msg.poses[0].pose.position.x))
        
        for waypoint_pose in msg.poses:
            # y must be negated for conversion from ros2 env to CARLA
            waypoint_position = carla.Location(x = waypoint_pose.pose.position.x, y = -waypoint_pose.pose.position.y, z = waypoint_pose.pose.position.z)
            self.world.debug.draw_string(waypoint_position, '^', draw_shadow=False,
                            color=carla.Color(r=0, g=0, b=255), life_time=30.0, persistent_lines=True)
        
            
        curr_wp = 2 #we will be tracking waypoints in the route and switch to next one when we get close to current one
        predicted_angle = 0
        while curr_wp<len(msg.poses)-1:
            #print("Inside FIRST while loop")
            self.waypoint_position = carla.Location(x = msg.poses[curr_wp].pose.position.x, y = -msg.poses[curr_wp].pose.position.y, z = msg.poses[curr_wp].pose.position.z)
            while curr_wp<len(msg.poses) and self.hero.get_transform().location.distance(self.waypoint_position)<5:
                #print("Inside SECOND while loop")
                curr_wp +=1 #move to next wp if we are too close
                if curr_wp < len(msg.poses):
                    self.waypoint_position = carla.Location(x = msg.poses[curr_wp].pose.position.x, y = -msg.poses[curr_wp].pose.position.y, z = msg.poses[curr_wp].pose.position.z)
            
            #print("curr_wp number: ", curr_wp)
            #print("waypoint_position: ", waypoint_position)
            predicted_angle = self.get_angle(self.hero, self.waypoint_position)
            #print("predicted_angle: ", predicted_angle)
            v = self.hero.get_velocity()
            self.speed = round(3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2),1)
            self.estimated_throttle = self.maintain_speed(self.speed)
            print("Vehicle speed: ", self.speed)

            #save vehicle speed to analyze
            '''
            self.csvwriter.writerow([self.speed])
            '''


            #print("estimated throttle: ", self.estimated_throttle)
            # extra checks on predicted angle when values close to 360 degrees are returned
            if predicted_angle<-300:
                predicted_angle = predicted_angle+360
            elif predicted_angle > 300:
                predicted_angle = predicted_angle -360
            steer_input = predicted_angle
            #and conversion of to -1 to +1
            if predicted_angle<-30:
                steer_input = -30
            elif predicted_angle>30:
                steer_input = 30
            # conversion from degrees to -1 to +1 input for apply control function
            self.steer_input = steer_input/75

            control_msg = CarlaEgoVehicleControl()
            control_msg.throttle = self.estimated_throttle
            control_msg.gear = 1
            control_msg.steer = self.steer_input
            control_msg.manual_gear_shift = bool(0)
            
            try:
                #print("Entering TRY module")
                #time.sleep(0.2)
                time.sleep(0.05)
                self.vehicle_control_publisher.publish(control_msg)
            except Exception as error:
                self.logerr("Could not send vehicle control: {}".format(error))

        try:
            control_msg.throttle = 0.3
            control_msg.steer = 0.0
            control_msg.brake = 0.0
            print("Entering TRY module")
            while self.hero.get_transform().location.distance(self.waypoint_position)>1.0:
                v = self.hero.get_velocity()
                self.speed = round(3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2),3)
                print("vehicle speed inside while loop: ", self.speed)
                time.sleep(0.05)
                self.vehicle_control_publisher.publish(control_msg)
            #self.vehicle_control_publisher.publish(control_msg)
            
            control_msg.throttle = 0.0
            control_msg.steer = 0.0
            control_msg.brake = 0.3
            v = self.hero.get_velocity()
            self.speed = round(3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2),3)
            while self.speed>=0.001:
                v = self.hero.get_velocity()
                self.speed = round(3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2),3)
                print("vehicle speed while braking: ", self.speed)
                time.sleep(0.05)
                self.vehicle_control_publisher.publish(control_msg)
            
            exit()
        except Exception as error:
            self.logerr("Could not send vehicle control: {}".format(error))


    def connect_to_carla(self):
        self.loginfo("Waiting for CARLA world (topic: /carla/world_info)...")
        try:
            self.wait_for_message(
                "/carla/world_info",
                CarlaWorldInfo,
                qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
                timeout=15.0)
        except ROSException as e:
            self.logerr("Error while waiting for world info: {}".format(e))
            raise e

        host = self.get_param("host", "127.0.0.1")
        port = self.get_param("port", 2000)
        timeout = self.get_param("timeout", 10)
        self.loginfo("CARLA world available. Trying to connect to {host}:{port}".format(
            host=host, port=port))

        carla_client = carla.Client(host=host, port=port)
        carla_client.set_timeout(timeout)

        try:
            self.world = carla_client.get_world()
        except RuntimeError as e:
            self.logerr("Error while connecting to Carla: {}".format(e))
            raise e

        self.loginfo("Connected to Carla.")


def main(args=None):
    roscomp.init('waypoint_subscriber', args)
    waypoint_subscriber = None
    try:
        waypoint_subscriber = WayPointSubscriber()
        waypoint_subscriber.spin()
    except (RuntimeError, ROSException):
        pass
    except KeyboardInterrupt:
        roscomp.loginfo("User requested shut down.")
    finally:
        roscomp.loginfo("Shutting down.")
        if waypoint_subscriber:
            waypoint_subscriber.destroy()
        roscomp.shutdown()
