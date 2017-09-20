from math import sqrt, cos, sin
import numpy as np
import math


def get_Euler_Angle(pose):
    """Returns the roll, pitch yaw angles from a Quaternion \
    Args:
        pose: geometry_msgs/Pose.msg
    Returns:
        roll (float), pitch (float), yaw (float)
    """
    ysqr = pose.orientation.y * pose.orientation.y

    t0 = +2.0 * (pose.orientation.w * pose.orientation.x + pose.orientation.y * pose.orientation.z)
    t1 = +1.0 - 2.0 * (pose.orientation.x * pose.orientation.x + ysqr)
    roll = math.degrees(math.atan2(t0, t1))

    t2 = +2.0 * (pose.orientation.w * pose.orientation.y - pose.orientation.z * pose.orientation.x)
    t2 = 1 if t2 > 1 else t2
    t2 = -1 if t2 < -1 else t2
    pitch = math.degrees(math.asin(t2))

    t3 = +2.0 * (pose.orientation.w * pose.orientation.z + pose.orientation.x * pose.orientation.y)
    t4 = +1.0 - 2.0 * (ysqr + pose.orientation.z * pose.orientation.z)
    yaw = math.degrees(math.atan2(t3, t4))

    return roll, pitch, yaw


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


def fit_polynomial(waypoints, degree):
    """fits a polynomial for given waypoints"""
    x_coords = [waypoint.pose.pose.position.x for waypoint in waypoints]
    y_coords = [waypoint.pose.pose.position.y for waypoint in waypoints]
    return np.polyfit(x_coords, y_coords, degree)


def calculateRCurve(coeffs, X):
    """calculates the radius of curvature"""
    if coeffs is None:
        return None
    a = coeffs[0]
    b = coeffs[1]
    return (1 + (2 * a * X + b) ** 2) ** 1.5 / np.absolute(2 * a)


def coordinate_transform_global_to_local(pose, waypoints, points_to_use=None):
    """
    From a pose object transfrom a series of waypoints from global
    coordinate to vehicle local coordinate
    Args:
        pose (object) : A pose object
        waypoints (list) : A list of waypoint objects
        points_to_use (int) : How many points to use (None => all)
    Returns:
        x_coords (list): The transformed x-coordinates of waypoints
        y_coords (list): The transformed y-coordinates of waypoints
    """
    x_coords = []
    y_coords = []

    _, _, yaw = get_Euler_Angle(pose)
    originX = pose.position.x
    originY = pose.position.y

    if (points_to_use is None) | (points_to_use >= len(waypoints)):
        points_to_use = len(waypoints)

    for i in range(points_to_use):
        shift_x = waypoints[i].pose.pose.position.x - originX
        shift_y = waypoints[i].pose.pose.position.y - originY

        x = shift_x * cos(0 - yaw) - shift_y * sin(0 - yaw)
        y = shift_x * sin(0 - yaw) + shift_y * cos(0 - yaw)

        x_coords.append(x)
        y_coords.append(y)

    return x_coords, y_coords


def cte(pose, waypoints, polynomial_order=3, evaluation_locaiton=3, points_to_fit=10):
    """
    Estimate trajectory tracking performance
    Args:
        pose (object) : A pose object
        waypoints (list) : A list of waypoint objects
        polynomial_order (int) : order of polynomial to fit waypoints
        evaluation_locaiton (float) : location to estimate cte
    Returns:
        cte(float) : distance from trajectory to pose, positive if
        vehicle located at rightside of trajectory
        yaw_error(float) : angular difference between current pose and trajectory,
        positive if yaw to right
    """
    if points_to_fit>len(waypoints):
        points_to_fit = len(waypoints)

    x_coords, y_coords = coordinate_transform_global_to_local(
        pose, waypoints, points_to_fit)
    coefficients = np.polyfit(x_coords, y_coords, polynomial_order)
    distance = np.polyval(coefficients, evaluation_locaiton)
    gradient = np.polyval(np.polyder(coefficients, 1), evaluation_locaiton)
    angle = -math.degrees(math.tan(gradient))

    return distance, angle