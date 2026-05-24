from devices import (
    robot, timestep, imu,
    fl_range, fr_range,
    width, OBSTACLE_THRESHOLD,
    detect_target, get_lidar_distances
)

from goal_reached import check_transition, run_goal_reached
from proxi_readings import run_proxi_readings
from nav_target import run_nav_target
from explore import run_explore
from recovery import run_recovery


# =====================================================
# SUPERVISOR (exact world coordinates)
# =====================================================

robot_node = robot.getSelf()

# =====================================================
# STATE MACHINE VARIABLES
# =====================================================

state = "EXPLORE"

goal_reached = False
target_coords = None

# -----------------------------------------------------
# RECOVERY VARIABLES
# -----------------------------------------------------

stuck_counter = 0
RECOVERY_TRIGGER = 60
recovery_active = False

position_check_timer = 0
last_check_x = 0.0
last_check_y = 0.0
POSITION_CHECK_INTERVAL = 100  # check every ~3.2 seconds
POSITION_STUCK_THRESHOLD = 0.05  # metres — less than 5cm moved = stuck

# =====================================================
# MAIN LOOP
# =====================================================

while robot.step(timestep) != -1:

    # =========================
    # POSITION + HEADING
    # =========================

    pos = robot_node.getField('translation').getSFVec3f()

    robot_x = pos[0]
    robot_y = pos[1]

    heading = imu.getRollPitchYaw()[2]

    # =========================
    # SENSOR READINGS
    # =========================

    fl_val = fl_range.getValue()
    fr_val = fr_range.getValue()

    front_dist, left_dist, right_dist = get_lidar_distances()

    obstacle = front_dist < OBSTACLE_THRESHOLD

    # =========================
    # TARGET DETECTION
    # =========================

    target_detected, target_x, red_pixel_count = detect_target()

    center = width / 2

    # =========================
    # STUCK DETECTION
    # =========================

    if obstacle:
        stuck_counter += 1
    else:
        stuck_counter = 0

    if not goal_reached and not recovery_active:
        position_check_timer += 1
        if position_check_timer >= POSITION_CHECK_INTERVAL:
            dist_moved = (robot_x - last_check_x) ** 2 + (robot_y - last_check_y) ** 2
            if dist_moved < POSITION_STUCK_THRESHOLD ** 2:
                recovery_active = True
            last_check_x = robot_x
            last_check_y = robot_y
            position_check_timer = 0

    # =========================
    # GOAL_REACHED TRANSITION
    # =========================

    if not goal_reached:

        transitioned, coords = check_transition(
            target_detected,
            red_pixel_count,
            fl_val,
            fr_val,
            target_x,
            center,
            robot_x,
            robot_y,
            heading
        )

        if transitioned:
            goal_reached = True
            target_coords = coords

    # =================================================
    # PRIORITY 0 : GOAL_REACHED
    # =================================================

    if goal_reached:

        state = "GOAL_REACHED"
        run_goal_reached()

    # =================================================
    # PRIORITY 1 : RECOVERY
    # =================================================

    elif recovery_active or stuck_counter > RECOVERY_TRIGGER:

        recovery_active = True

        state = "RECOVERY"

        recovered = run_recovery()

        if recovered:
            recovery_active = False
            stuck_counter = 0
            state = "EXPLORE"

    # =================================================
    # PRIORITY 2 : PROXI_READINGS
    # =================================================

    elif obstacle and not (target_detected and abs(target_x - center) < 100):

        state = "PROXI_READINGS"

        run_proxi_readings()

    # =================================================
    # PRIORITY 3 : NAV_TARGET
    # =================================================

    elif target_detected:

        state = "NAV_TARGET"

        run_nav_target(target_x, center)

    # =================================================
    # PRIORITY 4 : EXPLORE
    # =================================================

    else:

        state = "EXPLORE"

        run_explore()
