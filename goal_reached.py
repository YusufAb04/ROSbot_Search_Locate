import math
from devices import (
    fl, fr, rl, rr, stop,
    range_finder, width, height,
    GOAL_DISTANCE, GOAL_RED_THRESHOLD,
    CAMERA_X_OFFSET, BOX_HALF_SIZE,
    SENSOR_X_OFFSET
)

def check_transition(target_detected, red_pixel_count, fl_val, fr_val,
                     target_x, center, robot_x, robot_y, heading):
    if (target_detected and
            red_pixel_count >= GOAL_RED_THRESHOLD and
            min(fl_val, fr_val) < GOAL_DISTANCE and
            abs(target_x - center) < 50):

        # Sample depth at the red object's horizontal position, middle row.
        # The depth camera sits 0.027m behind the robot centre, so subtract
        # that offset to get distance from robot centre to the box surface.
        depth_image = range_finder.getRangeImage()
        pixel_idx   = (height // 2) * width + int(target_x)
        depth       = depth_image[pixel_idx]

        if math.isfinite(depth) and depth > 0:
            distance_to_centre = depth - CAMERA_X_OFFSET + BOX_HALF_SIZE
        else:
            # Fallback to proximity sensor if depth is invalid
            distance_to_centre = min(fl_val, fr_val) + SENSOR_X_OFFSET + BOX_HALF_SIZE

        coords = (
            round(robot_x + distance_to_centre * math.cos(heading), 3),
            round(robot_y + distance_to_centre * math.sin(heading), 3)
        )
        stop()
        print("=" * 50)
        print("STATE: GOAL_REACHED")
        print("TARGET FOUND!")
        print(f"  Coordinates -> x: {coords[0]} m | y: {coords[1]} m")
        print("=" * 50)
        return True, coords

    return False, None

def run_goal_reached():
    fl.setVelocity(-4.0)
    fr.setVelocity(4.0)
    rl.setVelocity(-4.0)
    rr.setVelocity(4.0)
