#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from itertools import islice, cycle
from std_msgs.msg import Int32

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
MAX_SPEED_METERS_PER_SEC = 30*0.447

class WaypointUpdater(object):
    def __init__(self):

        rospy.loginfo("waypoint_updater.py --> __init__")

        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Lane, self.obstacle_cb)

        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below


        self.waypoints = None
        self.pose = None
        self.frame_id = None
        self.velocity = None

        self.red_light_id = None

        #rospy.spin()
        self.mainiteration()

    def mainiteration(self):
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            self.publish_final_waypoints()

    def velocity_cb(self, msg):
        self.velocity = msg.twist.linear.x

    def pose_cb(self, msg):
        # TODO: Implement
        #rospy.loginfo("waypoint_updater.py --> pose_cb")
        self.pose = msg.pose
        self.frame_id = msg.header.frame_id
        #self.publish_final_waypoints()
        #pass

    def waypoints_cb(self, msg):
        # TODO: Implement
        #rospy.loginfo("waypoint_updater.py --> waypoints_cb")
        self.frame_id = msg.header.frame_id
        self.waypoints = msg.waypoints
        #self.publish_final_waypoints()

        #pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement

        self.red_light_id = msg.data

        #rospy.loginfo("traffic_cb:%d   ",msg.data)

        #pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def publish_final_waypoints(self):
        if self.waypoints is None or self.pose is None:
            return

        # Calculate which is the closest waypoint, so we start to publish from that
        dist_min = 99999999
        index_min = 0
        prev_index_min = 0
        carx = self.pose.position.x
        cary = self.pose.position.y

        for i in range(0, len(self.waypoints)):
            x = self.waypoints[i].pose.pose.position.x
            y = self.waypoints[i].pose.pose.position.y
            distsq = (carx - x)**2 + (cary - y)**2
            if (distsq < dist_min):
                dist_min = distsq
                prev_index_min = index_min
                index_min = i

        if prev_index_min - index_min > 100:
            prev_index_min = index_min  #circled
        index_min = max(index_min, prev_index_min) #ahead the one with bigger index
        waypoint_nearest = self.waypoints[index_min]

        waypoints_next = list(islice(cycle(self.waypoints), index_min, index_min + LOOKAHEAD_WPS))

        #rospy.loginfo("waypoint_updater.py -->vel:%f i:%d  dist:%f  len:%d", self.velocity, index_min, dist_min, len(waypoints_next))

        # Do we have red traffic light near by?

        acceleration = 0.
        start_speed = self.velocity
        if self.red_light_id > 0:
            waypoints_ahead = 10.
            waypoints_distance = max(1., self.red_light_id - index_min - waypoints_ahead)
            target_speed = 0.
            acceleration = (target_speed - start_speed)/waypoints_distance
            #rospy.loginfo("waypoint_updater.py RED tl_id:%d  wpo_id:%d v:%f dist:%f accel:%f", 
            #    self.red_light_id, index_min, start_speed, waypoints_distance, acceleration)


        else:
            waypoints_distance = 10.
            target_speed = MAX_SPEED_METERS_PER_SEC
            acceleration = (target_speed - start_speed)/waypoints_distance            
            #rospy.loginfo("waypoint_updater.py accel:%f",  acceleration)


        #rospy.loginfo("waypoint_updater.py  tl_id:%d  wpo_id:%d dist:%f accel:%f", self.red_light_id, index_min, waypoints_distance, acceleration)



        # Set the velocity
        speeds = []
        for i, waypoint in enumerate(waypoints_next):
            start_speed += acceleration
            start_speed = min(start_speed, MAX_SPEED_METERS_PER_SEC)
            start_speed = max(0., start_speed)
            if self.red_light_id > 0.:
                if start_speed < 1.0 or i > self.red_light_id:
                    start_speed = 0.0
            waypoints_next[i].twist.twist.linear.x = start_speed
            speeds.append(start_speed)
            #waypoints_next[i].twist.twist.linear.x = MAX_SPEED_METERS_PER_SEC
            #self.set_waypoint_velocity(waypoints_next, i, MAX_SPEED_METERS_PER_SEC)

        '''
        if self.red_light_id > 0:
            rospy.loginfo("wps: %s" , ''.join(str(x) for x in speeds))
        '''

        lane = Lane()
        lane.waypoints = waypoints_next
        lane.header.frame_id = self.frame_id
        lane.header.stamp = rospy.Time.now()
        self.final_waypoints_pub.publish(lane)


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
