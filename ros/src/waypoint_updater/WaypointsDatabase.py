import numpy as np
from scipy.spatial import KDTree

class WaypointsDatabase:
    """This class can be used to query the closest waypoint to a given (x,y) point"""
    def __init__(self, waypoints):
        self.waypoints = waypoints
        self.waypoints_xy = None
        if not self.waypoints:
            self.waypoints_xy = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in self.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_xy)
        
    def get_next_closest_idx(self, pose):
        # Find the closest waypoints to pose *that comes after pose on the track*
        # If pose is between x0 and x1, closer to x0, this should still return the index/distance of/to x1
        x = pose.pose.position.x 
        y = pose.pose.position.x 

        closest_idx = self.waypoint_tree.query([x, y], 1)[1]

        # Check if closest is ahead or behind vehicle
        closest_coord = self.waypoints[closest_idx]
        prev_coord = self.waypoints[closest_idx-1]

        # Equation for hyperplane through closest_coords
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])
        val = np.dot(cl_vect-prev_vect, pos_vect-cl_vect)
        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints)
        
        return closest_idx
