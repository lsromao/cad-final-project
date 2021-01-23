import numpy as np
from scipy.spatial import KDTree
import rospy

class WaypointsDatabase:
    """This class can be used to query the closest waypoint to a given (x,y) point"""
    def __init__(self, waypoints):
        self.waypoints = waypoints
        self.waypoints_xy = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in self.waypoints]
        self.waypoint_tree = KDTree(self.waypoints_xy)

    def get_next_closest_idx(self, pose):
        # Find the closest waypoints to pose *that comes after pose on the track*
        # If pose is between x0 and x1, closer to x0, this should still return the index/distance of/to x1
        closest_idx = self.waypoint_tree.query([pose[0], pose[1]], 1)[1]

        # Check if closest is ahead or behind vehicle
        closest_coord = self.waypoints_xy[closest_idx]
        prev_coord = self.waypoints_xy[closest_idx-1]

        # Equation for hyperplane through closest_coords
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([pose[0], pose[1]])
        val = np.dot(cl_vect-prev_vect, pos_vect-cl_vect)
        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_xy)
        
        return closest_idx

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist
