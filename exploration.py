from robot import Robot
from utils import *
import math


def get_wheel_velocities(robbie, coord):
    """
    Helper function to determine the velocities of the robot's left and right wheels.
    Arguments:
        robbie: instance of the robot
        coord (tuple): coordinate to move to (x,y)

    Returns: 
        vr, vl: velocities of the robot's left and right wheels
    """

    # Calculate the desired change in position
    dx_world = coord[0] - robbie.x
    dy_world = coord[1] - robbie.y
    dx_robot, dy_robot = rotate_point(dx_world, dy_world, robbie.h)
    dist_to_coord = math.sqrt(dx_robot**2 + dy_robot**2)

    # Turn in place first
    angle = math.atan2(dy_robot, dx_robot)
    threshold = 0.1
    if angle < -threshold:
        return -0.01, 0.01
    elif angle > threshold:
        return 0.01, -0.01

    # Using desired linear velocity, set left and right wheel velocity
    linear_v = 0.05 * dist_to_coord
    w = 0.3 * math.atan2(dy_robot, dx_robot)
    vl = (linear_v - robbie.wheel_dist / 2 * w)
    vr = (linear_v + robbie.wheel_dist / 2 * w)
    return vr, vl


def frontier_planning(robbie, grid):
    """
        OPTIONAL: Function for defining frontier planning.

        Arguments:
            robbie: instance of the robot
            grid: instance of the grid

        Returns:
            robbie: 'updated' instance of the robot
            OPTIONAL: robbie.next_coord: new destination coordinate

        Notes:
            The lecture notes should provide you with an ample description of frontier planning.
            You will also find many of the functions declared in 'grid.py' and 'utils.py' useful.

    """
    ### TODO: Student Code here ###
    frontiers = find_frontiers(robbie, grid)
    if not frontiers:
        return None  # No more frontiers

    frontier_groups = group_frontiers(frontiers, grid)
    if not frontier_groups:
        return None  # No valid frontier groups found

    best_frontier = select_best_frontier(frontier_groups, robbie)
    if best_frontier is None:
        return None

    centroid_of_best_frontier = centroid(best_frontier)
    robbie.next_coord = centroid_of_best_frontier
    return grid.rrt(robbie.xy, centroid_of_best_frontier)


def exploration_state_machine(robbie, grid):
    """
    Use frontier planning, or another exploration algorithm, to explore the grid.

    Arguments:
        robbie: instance of the robot
        grid: instance of the grid

    Returns: 
        robbie: 'updated' instance of the robot

    Notes:
        Robot is considered as Point object located at the center of the traingle.
        You may use the 'rrt' function (see grid.py) to find a new path whenever the robot encounters an obstacle.
        You can use 'grid.is_collision_with_obstacles()' to check if the robot encounters an obstacle.
        Please note that the use of rrt slows down your code, so it should be used sparingly.
        The 'get_wheel_velocities' functions is useful in setting the robot's velocities.
        You will also find many of the functions declared in 'grid.py' and 'utils.py' useful.
        Feel free to create other helper functions (in this file) as necessary.

    Alert:
        In this part, the task is to let the robot find all markers by exploring the map,
        which means using 'grid.markers' will lead  cause zero point on GraderScope.

    """
    ### TODO: Student Code here ###
    if not hasattr(robbie, 'path') or not robbie.path:
        robbie.path = frontier_planning(robbie, grid)
        if not robbie.path:
            # No path available, initiate rotation or other recovery mechanism
            initiate_rotation(robbie)
            return robbie

    next_coord = robbie.path[0]
    if is_at_position(robbie.xy, next_coord):
        # Reached the next point in the path, remove it from the path list
        robbie.path.pop(0)
        if not robbie.path:
            # Path is empty, recalculate path
            robbie.path = frontier_planning(robbie, grid)
            return robbie

    vr, vl = get_wheel_velocities(robbie, next_coord)
    robbie.vr, robbie.vl = vr, vl
    # Add a check to ensure that the robot does not collide with obstacles
    if grid.is_collision_with_obstacles(robbie.xy, next_coord):
        robbie.path = frontier_planning(robbie, grid)
        if not robbie.path:
            # No path available, initiate rotation or other recovery mechanism
            initiate_rotation(robbie)
            return robbie
    robbie.move_diff_drive(grid, vl, vr, robbie.TIMESTEP)
    return robbie


def is_at_position(current_pos, target_pos, tolerance=0.1):
    """
    Check if the robot is at the target position within a specified tolerance.
    """
    return distance(current_pos, target_pos) <= tolerance


def initiate_rotation(robbie):
    # Simple rotation logic
    robbie.vr, robbie.vl = -0.1, 0.1


def find_frontiers(robbie, grid):
    explored = robbie.explored_cells
    frontiers = set()

    for x, y in explored:
        for dx in range(-1, 2):
            for dy in range(-1, 2):
                nx, ny = x + dx, y + dy
                if (nx, ny) not in explored and grid.is_in(nx, ny) and not grid.is_occupied(nx, ny):
                    frontiers.add((nx, ny))

    return list(frontiers)


def group_frontiers(frontiers, grid):
    frontier_groups = []
    for frontier in frontiers:
        found_group = False
        for group in frontier_groups:
            if any(grid_distance(frontier[0], frontier[1], x, y) <= 1 for x, y in group):
                group.append(frontier)
                found_group = True
                break
        if not found_group:
            frontier_groups.append([frontier])

    return frontier_groups


def select_best_frontier(frontier_groups, robbie):
    if not frontier_groups:
        return None

    return max(frontier_groups, key=len)


def centroid(cells):
    x_coords, y_coords = zip(*cells)
    return sum(x_coords) / len(cells), sum(y_coords) / len(cells)


def distance(point1, point2):
    return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)
