from grid import *
from robot import *
import time
import math
from utils import *


def get_wheel_velocities(robbie, coord, pickup_marker=False, threshold=0.1):
    """
    Helper function to determine the velocities of the robot's left and right wheels.
    Arguments:
        robbie: instance of the robot
        coord (tuple): coordinate to move to (x,y)
        pickup_marker (bool): Only set to 'True' when picking up marker
        threshold (int): Set to expected heading when trying to align robot with marker

    Returns: 
        vr, vl: velocities of the robot's left and right wheels
    """

    # Calculate the desired change in position
    controller = PidController()
    dx_world = coord[0] - robbie.x
    dy_world = coord[1] - robbie.y
    dx_robot, dy_robot = rotate_point(dx_world, dy_world, robbie.h)

    # Turn in place
    if not pickup_marker:
        angle = math.atan2(dy_robot, dx_robot)
        if angle < -threshold:
            return -0.02, 0.02
        elif angle > threshold:
            return 0.02, -0.02
    else:
        angle = robbie.h
        if angle < threshold:
            return 0.02, -0.02
        elif angle > threshold:
            return -0.02, 0.02

    robot_pose = np.array([robbie.xyh[0], robbie.xyh[1], robbie.xyh[2]])
    goalpoint = np.array([coord[0], coord[1]])
    linear_v = controller.linear_controller(robot_pose, goalpoint)
    w = controller.angular_controller(robot_pose, goalpoint)
    vl = (linear_v - robbie.wheel_dist / 2 * w)
    vr = (linear_v + robbie.wheel_dist / 2 * w)
    return vr, vl


def phase2_planning(robbie, grid):
    """
    This function should move the robot from it's starting position to a marker and then 'pick up' the marker.
    Arguments:
        robbie: instance of robot class
        grid: instance of grid class
    Returns:
        robbie: 'updated' instance of robot class
    Notes:
        Markers for each grid can be accessed through grid.markers
        Sample Pseudocode (this is just to give you an idea of how to implement this function. It MAY NOT be a complete solution):
        1. Move the robot from its current position to a marker on the grid. Use the 'get_wheel_velocities' function to determine the robot's velocities. 
           Note that the 'get_wheel_velocities' function relies on your PIDController implementation in utils.py.
           You may use the 'rrt' function (see grid.py) when the robot encounters an obstacle.
        2. When the robot reaches a marker, it must orient itself in the same orientation as the marker so as to 'pick it up'.
           For example if the marker's orientation is 'R', once the robot gets to the marker, it should turn in place till it's heading is 0 degrees.
           The 'get_wheel_velocities' function may be used to accomplish this. Note that you must set the 'pickup_marker' variable to 'True' when calling it. 
           Threshold would also need to be set to the appropriate heading for each marker.
           The expected heading for each of the markers can be accessed by calling the 'parse_marker_info' function in grid.py
        3. You may keep track of rrt path (if using rrt) by storing in the 'path' function member and current marker by storing in the 'curr_marker' function member
           in the robot class (check robot.py).

    Alert:
        In this part, the robot is expected to 'pick up' all markers by going the desired locations.
        You may call 'grid.markers' to get the markers' coordinates. 
        However, modifying elements in 'robot.markers_found_or_picked' is prohibited.

    """
    if should_find_new_marker(robbie, grid):
        robbie.curr_marker = find_closest_unvisited_marker(robbie, grid)

    if should_recalculate_path(robbie, grid):
        recalculate_path(robbie, grid)

    move_robot_to_next_coord(robbie, grid)

    return robbie


def should_find_new_marker(robbie, grid):
    return robbie.curr_marker is None or robbie.marker_already_picked(robbie.curr_marker)


def find_closest_unvisited_marker(robbie, grid):
    unvisited = [
        m for m in grid.markers if not robbie.marker_already_picked(m)]
    if not unvisited:
        return None
    dists = [grid_distance(robbie.x, robbie.y, m[0], m[1]) for m in unvisited]
    return unvisited[dists.index(min(dists))]


def should_recalculate_path(robbie, grid):
    return robbie.path is None or not robbie.path or grid.is_collision_with_obstacles(robbie.xy, robbie.path[0])


def recalculate_path(robbie, grid):
    marker = list(robbie.curr_marker)
    marker[0] += 0.5
    marker[1] += 0.5
    robbie.path = grid.rrt(robbie.xy, tuple(marker))[1:]


def move_robot_to_next_coord(robbie, grid):
    if grid_distance(robbie.x, robbie.y, robbie.path[0][0], robbie.path[0][1]) <= 0.05:
        robbie.next_coord = robbie.path.pop(0)
    else:
        robbie.next_coord = robbie.path[0]

    vr, vl = get_wheel_velocities(robbie, robbie.next_coord)
    robbie.vr = vr
    robbie.vl = vl
    robbie.move_diff_drive(grid, vl, vr, robbie.TIMESTEP)
