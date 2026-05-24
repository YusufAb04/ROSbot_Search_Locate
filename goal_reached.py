import math
from devices import (
    camera, fl, fr, rl, rr, stop,
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

        # True bearing to the target — heading plus the horizontal angle offset
        # from the camera centre to the red pixel centroid.
        fov_h = camera.getFov()
        angle_to_target = (center - target_x) / (width / 2) * (fov_h / 2)
        target_bearing = heading + angle_to_target

        # Scan the full column at target_x for the minimum depth value.
        # At close range the box appears near the bottom of the image, so
        # sampling only the middle row reads the background wall instead.
        depth_image = range_finder.getRangeImage()
        tx = int(target_x)
        min_depth = float('inf')
        for row in range(height):
            d = depth_image[row * width + tx]
            if math.isfinite(d) and d > 0:
                min_depth = min(min_depth, d)

        prox_dist = min(fl_val, fr_val)

        # Use depth camera only when its reading is consistent with the
        # proximity sensor; otherwise the depth is reading background.
        if min_depth < prox_dist + SENSOR_X_OFFSET + 0.20:
            distance_to_centre = min_depth - CAMERA_X_OFFSET + BOX_HALF_SIZE
        else:
            distance_to_centre = prox_dist + SENSOR_X_OFFSET + BOX_HALF_SIZE

        coords = (
            round(robot_x + distance_to_centre * math.cos(target_bearing), 3),
            round(robot_y + distance_to_centre * math.sin(target_bearing), 3)
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
