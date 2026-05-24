# explore.py


import math

from devices import (
    robot,
    imu,
    forward,
    turn_left,
    turn_right,
    stop
)

# =====================================================
# GRID SETTINGS
# =====================================================

GRID_SIZE = 0.5   # metres per cell

# =====================================================
# SUPERVISOR ACCESS
# =====================================================

robot_node = robot.getSelf()

# =====================================================
# EXPLORE STATE VARIABLES
# =====================================================

visited_cells = set()

explore_mode = "FORWARD"

turn_target_heading = 0

# =====================================================
# HELPER FUNCTIONS
# =====================================================


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2 * math.pi

    while angle < -math.pi:
        angle += 2 * math.pi

    return angle



def get_current_cell(robot_x, robot_y):
    grid_x = int(robot_x / GRID_SIZE)
    grid_y = int(robot_y / GRID_SIZE)

    return (grid_x, grid_y)



def get_heading_direction(heading):

    heading = normalize_angle(heading)

    # EAST
    if -0.785 <= heading < 0.785:
        return "EAST"

    # NORTH
    elif 0.785 <= heading < 2.355:
        return "NORTH"

    # SOUTH
    elif -2.355 <= heading < -0.785:
        return "SOUTH"

    # WEST
    else:
        return "WEST"



# =====================================================
# NEXT CELL SELECTION
# =====================================================


def get_next_cell(current_cell, direction):

    if direction == "EAST":
        return (current_cell[0] + 1, current_cell[1])

    elif direction == "WEST":
        return (current_cell[0] - 1, current_cell[1])

    elif direction == "NORTH":
        return (current_cell[0], current_cell[1] + 1)

    else:
        return (current_cell[0], current_cell[1] - 1)



def choose_best_direction(current_cell):

    directions = [
        ("EAST", 0),
        ("NORTH", math.pi / 2),
        ("WEST", math.pi),
        ("SOUTH", -math.pi / 2)
    ]

    # Prefer unexplored cells
    for direction_name, heading in directions:

        next_cell = get_next_cell(current_cell, direction_name)

        if next_cell not in visited_cells:
            return heading

    # If everything nearby explored,
    # continue normally
    return None


# =====================================================
# MAIN EXPLORE STATE
# =====================================================


def run_explore():

    global explore_mode
    global turn_target_heading

    print("STATE: EXPLORE")

    # =================================================
    # GET POSITION + HEADING
    # =================================================

    pos = robot_node.getField('translation').getSFVec3f()

    robot_x = pos[0]
    robot_y = pos[1]

    heading = imu.getRollPitchYaw()[2]

    # =================================================
    # CURRENT GRID CELL
    # =================================================

    current_cell = get_current_cell(robot_x, robot_y)

    visited_cells.add(current_cell)

    # =================================================
    # FORWARD EXPLORATION
    # =================================================

    if explore_mode == "FORWARD":

        forward(3.0)

        current_direction = get_heading_direction(heading)

        next_cell = get_next_cell(current_cell, current_direction)

        # If next cell already explored,
        # search for an unexplored neighbour
        if next_cell in visited_cells:

            best_heading = choose_best_direction(current_cell)

            # Only turn if an unexplored cell exists
            if best_heading is not None:

                stop()

                turn_target_heading = best_heading

                explore_mode = "TURN"

            # Otherwise continue forward normally

    # =================================================
    # TURNING MODE
    # =================================================

    elif explore_mode == "TURN":

        angle_error = normalize_angle(
            turn_target_heading - heading
        )

        if abs(angle_error) < 0.12:

            stop()

            explore_mode = "FORWARD"

        else:

            if angle_error > 0:
                turn_left()
            else:
                turn_right()
