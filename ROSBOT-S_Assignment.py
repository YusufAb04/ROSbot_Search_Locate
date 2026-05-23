import math
from devices import (
    robot, timestep, imu,
    fl_enc, fr_enc, rl_enc, rr_enc,
    fl_range, fr_range,
    width, WHEEL_RADIUS, OBSTACLE_THRESHOLD,
    detect_target
)
from goal_reached   import check_transition, run_goal_reached
from proxi_readings import run_proxi_readings
from nav_target     import run_nav_target
from explore        import run_explore

# =====================================================
# ODOMETRY STATE
# =====================================================

robot_x      = 0.0
robot_y      = 0.0
prev_fl_enc  = 0.0
prev_fr_enc  = 0.0
prev_rl_enc  = 0.0
prev_rr_enc  = 0.0

# =====================================================
# STATE MACHINE VARIABLES
# =====================================================

state          = "EXPLORE"
goal_reached   = False
target_coords  = None
turn_timer     = 0
turn_direction = None

# =====================================================
# MAIN LOOP
# =====================================================

while robot.step(timestep) != -1:

    # =========================
    # ODOMETRY UPDATE
    # =========================

    cur_fl_enc = fl_enc.getValue()
    cur_fr_enc = fr_enc.getValue()
    cur_rl_enc = rl_enc.getValue()
    cur_rr_enc = rr_enc.getValue()

    left_dist  = ((cur_fl_enc - prev_fl_enc) + (cur_rl_enc - prev_rl_enc)) / 2 * WHEEL_RADIUS
    right_dist = ((cur_fr_enc - prev_fr_enc) + (cur_rr_enc - prev_rr_enc)) / 2 * WHEEL_RADIUS
    dist       = (left_dist + right_dist) / 2

    heading = imu.getRollPitchYaw()[2]

    robot_x += dist * math.cos(heading)
    robot_y += dist * math.sin(heading)

    prev_fl_enc = cur_fl_enc
    prev_fr_enc = cur_fr_enc
    prev_rl_enc = cur_rl_enc
    prev_rr_enc = cur_rr_enc

    # =========================
    # SENSOR READINGS
    # =========================

    fl_val = fl_range.getValue()
    fr_val = fr_range.getValue()

    obstacle = fl_val < OBSTACLE_THRESHOLD or fr_val < OBSTACLE_THRESHOLD

    if turn_timer > 0:
        turn_timer -= 1

    target_detected, target_x, red_pixel_count = detect_target()
    center = width / 2

    # =========================
    # GOAL_REACHED TRANSITION
    # =========================

    if not goal_reached:
        transitioned, coords = check_transition(
            target_detected, red_pixel_count,
            fl_val, fr_val,
            target_x, center,
            robot_x, robot_y, heading
        )
        if transitioned:
            goal_reached  = True
            target_coords = coords

    # =========================
    # PRIORITY 0: GOAL_REACHED
    # =========================

    if goal_reached:
        state = "GOAL_REACHED"
        run_goal_reached()

    # =========================
    # PRIORITY 1: PROXI
    # =========================

    elif obstacle or turn_timer > 0:
        state = "PROXI_READINGS"
        turn_timer, turn_direction = run_proxi_readings(
            fl_val, fr_val, turn_timer, turn_direction
        )

    # =========================
    # PRIORITY 2: NAV_TARGET
    # =========================

    elif target_detected:
        state = "NAV_TARGET"
        run_nav_target(target_x, center)

    # =========================
    # PRIORITY 3: EXPLORE
    # =========================

    else:
        state = "EXPLORE"
        run_explore()
