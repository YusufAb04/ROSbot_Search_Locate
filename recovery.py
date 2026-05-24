# recovery.py
#
# RECOVERY STATE
#
# Purpose:
# Recover when the robot becomes stuck against a wall/obstacle.
#
# Strategy:
# 1. Reverse away from obstacle
# 2. Spin until clear AND turned at least 120 degrees from starting heading
#    to prevent returning to the same blocked path

import math

from devices import (
    turn_left,
    stop,
    fl, fr, rl, rr,
    fl_range, fr_range,
    imu,
    OBSTACLE_THRESHOLD
)

# =====================================================
# CONSTANTS
# =====================================================

MIN_SPIN_ANGLE = math.pi / 2  # 90 degrees minimum spin before accepting clear path

# =====================================================
# RECOVERY STATE VARIABLES
# =====================================================

recovery_step = 0
recovery_timer = 0
recovery_start_heading = 0.0

# =====================================================
# HELPER
# =====================================================

def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

# =====================================================
# BASIC MOVEMENTS
# =====================================================

def reverse(speed=3.5):
    fl.setVelocity(-speed)
    fr.setVelocity(-speed)
    rl.setVelocity(-speed)
    rr.setVelocity(-speed)

# =====================================================
# RECOVERY LOGIC
# =====================================================

def run_recovery():
    global recovery_step
    global recovery_timer
    global recovery_start_heading

    print("STATE: RECOVERY")

    fl_val = fl_range.getValue()
    fr_val = fr_range.getValue()

    # =================================================
    # STEP 1 -> REVERSE
    # =================================================

    if recovery_step == 0:

        recovery_start_heading = imu.getRollPitchYaw()[2]

        reverse(3.5)
        recovery_timer += 1

        if recovery_timer >= 30:
            recovery_timer = 0
            recovery_step = 1

    # =================================================
    # STEP 2 -> SPIN UNTIL CLEAR AND TURNED 120+
    # =================================================

    elif recovery_step == 1:

        current_heading = imu.getRollPitchYaw()[2]
        angle_turned = abs(normalize_angle(current_heading - recovery_start_heading))

        path_clear = fl_val > OBSTACLE_THRESHOLD and fr_val > OBSTACLE_THRESHOLD
        turned_enough = angle_turned >= MIN_SPIN_ANGLE

        if path_clear and turned_enough:
            stop()
            recovery_step = 0
            recovery_timer = 0
            return True

        turn_left()

    return False
