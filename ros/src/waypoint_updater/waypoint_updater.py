#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from WaypointsDatabase import WaypointsDatabase
import numpy as np
from scipy.spatial import KDTree
import math

'''
This node will publish waypoints ahead of the car's current position.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.
'''

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        # Subscribe to input topics
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('/base_waypoints', Lane, self.track_waypoints_callback)
        rospy.Subscriber("/traffic_waypoints", Int32, self.next_traffic_light_waypoint_callback)

        # Publisher for computed final waypoints
        self.final_waypoints_publisher = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        self.N = 100 # Number of waypoints to publish (planning horizon)
        self.max_deceleration = 0.5
        self.base_lane = None
        self.pose = None

        # Wait until the required info has been received
        rate = rospy.Rate(50) # 50Hz loop
        self.current_car_position = None
        self.waypoints_db = None
        self.next_traffic_light_stopline_index = -1
        rospy.loginfo("waiting for initial waypoint")
        while not rospy.is_shutdown():
            if self.current_car_position is not None and self.waypoints_db is not None:
                break
            rate.sleep()
        # Main loop for the node, running at a fixed rate
        while not rospy.is_shutdown():
            self.process()
            rate.sleep()
        
    def pose_callback(self, msg: PoseStamped):
        self.current_car_position = np.array([msg.pose.position.x, msg.pose.position.y]) # XYZ position of the car
        self.pose = msg


    def track_waypoints_callback(self, msg: Lane):
        self.waypoints_db = WaypointsDatabase(msg.waypoints)
    
    def next_traffic_light_waypoint_callback(self, msg: Int32):
        self.next_traffic_light_stopline_index = msg.data
    
    def process(self):
        # TODO: Use self.final_waypoints_pub to publish the next target waypoints
        # In phase 1: we can ignore traffic lights and simply output the next N waypoints *ahead of the car*, with their default velocity
        # In phase 2: you need to adjust target speeds on waypoints in order to smoothly brake until the car reaches the waypoint
        # corresponding to the next red light's stop line (stored in self.next_traffic_light_stopline_index, == -1 if no next traffic light).
        # Advice: make sure to complete dbw_node and have the car driving correctly while ignoring traffic lights before you tackle phase 2 
        msg = Lane()

        closest_idx =  self.waypoints_db.get_next_closest_idx(self.current_car_position)
        farthest_idx = closest_idx + self.N
        base_waypoints = self.waypoints_db.waypoints[closest_idx:farthest_idx]

        if self.next_traffic_light_stopline_index == -1 or (self.next_traffic_light_stopline_index >= farthest_idx):
            msg.waypoints = base_waypoints
        else:
            new_points = [] 
            for i, wp in enumerate(base_waypoints):
                point = Waypoint()
                point.pose = wp.pose

                stop_idx = max(self.next_traffic_light_stopline_index - closest_idx - 4, 0)
                dist = self.waypoints_db.distance(base_waypoints, i, stop_idx)
                vel = math.sqrt(2* self.max_deceleration * 0.90 * dist)
                if vel < 1.0:
                    vel = 0.0
                
                point.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
                new_points.append(point)
                
            msg.waypoints = new_points

        self.final_waypoints_publisher.publish(msg)

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except Exception as ex:
        rospy.logerr('Could not start waypoint updater node.')
        raise ex
