from devices import turn_left, turn_right, get_lidar_distances

def run_proxi_readings():
    print("STATE: PROXI_READINGS")

    _, left_dist, right_dist = get_lidar_distances()

    # Turn toward the more open side
    if left_dist >= right_dist:
        turn_left()
    else:
        turn_right()
