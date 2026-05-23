from devices import turn_left, turn_right, forward

def run_nav_target(target_x, center):
    print("STATE: NAV_TARGET")

    if target_x < center - 50:
        turn_left()
    elif target_x > center + 50:
        turn_right()
    else:
        forward(5.0)
