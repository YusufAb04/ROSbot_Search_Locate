from devices import OBSTACLE_THRESHOLD, turn_left, turn_right

def run_proxi_readings(fl_val, fr_val, turn_timer, turn_direction):
    print("STATE: PROXI_READINGS")

    if turn_timer <= 0:
        if fl_val < OBSTACLE_THRESHOLD and fr_val < OBSTACLE_THRESHOLD:
            turn_direction = "LEFT"
            turn_timer     = 50
        elif fl_val < OBSTACLE_THRESHOLD:
            turn_direction = "RIGHT"
            turn_timer     = 40
        elif fr_val < OBSTACLE_THRESHOLD:
            turn_direction = "LEFT"
            turn_timer     = 40

    if turn_direction == "LEFT":
        turn_left()
    elif turn_direction == "RIGHT":
        turn_right()

    return turn_timer, turn_direction
