from math import cos, sin, sqrt
import tf
import rospy
import numpy as np
import math

from styx_msgs.msg import Lane


def distance(waypoints, p1, p2):
    """ Get total distance between two waypoints given their index"""
    gap = 0
    for i in range(p1, p2 + 1):
        gap += get_distance(waypoints[p1].pose.pose.position, waypoints[i].pose.pose.position)
        p1 = i
    return gap


def make_lane_object(frame_id, waypoints):
    """Lane object contains the list of final waypoints ahead with velocity"""
    lane = Lane()
    lane.header.frame_id = frame_id
    lane.waypoints = waypoints
    lane.header.stamp = rospy.Time.now()
    return lane


def get_Euler_Angle(pose):
    """Returns the roll, pitch yaw angles from a Quaternion """
    return tf.transformations.euler_from_quaternion([pose.orientation.x,
                                                     pose.orientation.y,
                                                     pose.orientation.z,
                                                     pose.orientation.w])


def get_distance(a, b):
    """Returns distance between two points"""
    dx = a.x - b.x
    dy = a.y - b.y
    return sqrt(dx * dx + dy * dy)


def is_waypoint_front(pose, waypoint):
    """Take a waypoint and a pose , do a coordinate system transformation
    setting the origin at the position of the pose object and as x-axis
    the orientation of the z-axis of the pose
    Args:
        pose (object) : A pose object
        waypoints (object) : A waypoint object
    Returns:
        bool : True if the waypoint is behind the car False if in front
    """
    _, _, pose_yaw = get_Euler_Angle(pose)
    pose_X = pose.position.x
    pose_Y = pose.position.y

    waypoint_X = waypoint.pose.pose.position.x
    waypoint_Y = waypoint.pose.pose.position.y

    shift_x = waypoint_X - pose_X
    shift_y = waypoint_Y - pose_Y

    return (shift_x * cos(0 - pose_yaw) - shift_y * sin(0 - pose_yaw)) > 0


def get_closest_waypoint_index(pose, waypoints):
    """
    pose: geometry_msg.msgs.Pose instance
    waypoints: list of styx_msgs.msg.Waypoint instances
    returns index of the closest waypoint in the list waypoints
    """
    best_gap = float('inf')
    best_index = 0
    my_position = pose.position

    for i, waypoint in enumerate(waypoints):

        other_position = waypoint.pose.pose.position
        gap = get_distance(my_position, other_position)

        if gap < best_gap:
            best_index, best_gap = i, gap

    is_behind = ~is_waypoint_front(pose, waypoints[best_index])
    if is_behind:
        best_index += 1
    return best_index


def get_next_waypoints(waypoints, i, n):
    """Returns a list of n waypoints ahead of the vehicle"""
    m = min(len(waypoints), i + n)
    return waypoints[i:m]


def fit_polynomial(waypoints, degree):
    """fits a polynomial for given waypoints"""
    x_coords = [waypoint.pose.pose.position.x for waypoint in waypoints]
    y_coords = [waypoint.pose.pose.position.y for waypoint in waypoints]
    return np.polyfit(x_coords, y_coords, degree)


def calculateRCurve(coeffs, X):
    """
    calculates the radius of curvature
    Args:
        coeffs (vector) :polyfit coefficient of waypoints
        X (1D np array) : location to evaluate radius of curvature
    Return:
        radius_output (1D np array) : radius of curvature for X
    """
    if coeffs is None:
        return None
    coeffs_diff_1 = np.polyder(coeffs, 1)
    coeffs_diff_2 = np.polyder(coeffs, 2)

    radius_output = np.zeros(X.shape[0])
    for x_index in range(X.shape[0]):
        individual_x = X[x_index]
        radius = (1 + (np.polyval(coeffs_diff_1, individual_x) ** 2) ** 1.5) \
                 / np.absolute(np.polyval(coeffs_diff_2, individual_x))
        radius_output[x_index] = radius
    return radius_output
