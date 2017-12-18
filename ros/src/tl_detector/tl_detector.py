#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math
import random


STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        self.imagecounter = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints.waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        

        #rospy.loginfo("light position %d  %d  %d  %d:", self.state, state, self.state_count, light_wp)

        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        
        if self.waypoints is None:
            #rospy.loginfo("No waypoints . get_closest_waypoint")
            return

        min_gap = float('inf')
        min_index = 0

        for i, waypoint in enumerate(self.waypoints):
            gap = math.sqrt( (pose.position.x - waypoint.pose.pose.position.x)**2 + (pose.position.y - waypoint.pose.pose.position.y)**2 )
            if (gap < min_gap):
                min_index, min_gap = i, gap

        return min_index

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        '''
        if(round(random.uniform(1,100)<=4)):
            self.imagecounter = self.imagecounter + 1
            filename = '../../camera/image.' + str(self.imagecounter) + '.png'
            filename = 'image.' + str(self.imagecounter) + '.png'
            cv2.imwrite(filename,cv_image)
            rospy.loginfo("New image in: %s", filename)
        '''
        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None
        light_wp = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)
        else:
            return -1, TrafficLight.UNKNOWN

        #TODO find the closest visible traffic light (if one exists)
        # Iterate through all traffic lights and see which is in front of the car and with the smallest index
        light_wp_temp = 99999999

        #rospy.loginfo("Car position wp index %s", car_position)

        for light_pos in stop_line_positions:
            light_pose = Pose()
            light_pose.position.x = light_pos[0]
            light_pose.position.y = light_pos[1]
            light_wp = self.get_closest_waypoint(light_pose)

            if light_wp >= car_position:  # waypoint is ahead
                if light_wp < light_wp_temp:
                    light_wp_temp = light_wp 
                    light = light_pose

        light_wp = light_wp_temp

        if light_wp is not None and car_position is not None:
            dist = light_wp - car_position
        else:
            dist = 99999999

        #rospy.loginfo("Closest light position (in Wp index): %s  and dist: %s", light_wp, dist)

        if light and dist < 100:

            state = self.get_light_state(light)
            '''
            if state==TrafficLight.RED:
                rospy.loginfo("light position wp index %s color RED/YELLOW:", light_wp)
            else:
                rospy.loginfo("light position wp index %s color UNKNOWN:", light_wp)
            '''
            if state==TrafficLight.RED:
                return light_wp, TrafficLight.RED
            #return light_wp, TrafficLight.UNKNOWN

        #self.waypoints = None
        #return None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
