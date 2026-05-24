import math
from controller import Supervisor

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

# =====================================================
# MOTORS
# =====================================================

fl = robot.getDevice('fl_wheel_joint')
fr = robot.getDevice('fr_wheel_joint')
rl = robot.getDevice('rl_wheel_joint')
rr = robot.getDevice('rr_wheel_joint')

motors = [fl, fr, rl, rr]

for m in motors:
    m.setPosition(float('inf'))
    m.setVelocity(0.0)

# =====================================================
# WHEEL ENCODERS
# =====================================================

fl_enc = robot.getDevice('front left wheel motor sensor')
fr_enc = robot.getDevice('front right wheel motor sensor')
rl_enc = robot.getDevice('rear left wheel motor sensor')
rr_enc = robot.getDevice('rear right wheel motor sensor')

for enc in [fl_enc, fr_enc, rl_enc, rr_enc]:
    enc.enable(timestep)

# =====================================================
# IMU
# =====================================================

imu = robot.getDevice('imu inertial_unit')
imu.enable(timestep)

# =====================================================
# PROXIMITY SENSORS
# =====================================================

fl_range = robot.getDevice('fl_range')
fr_range = robot.getDevice('fr_range')
rl_range = robot.getDevice('rl_range')
rr_range = robot.getDevice('rr_range')

for s in [fl_range, fr_range, rl_range, rr_range]:
    s.enable(timestep)

# =====================================================
# CAMERA
# =====================================================

camera = robot.getDevice('camera rgb')
camera.enable(timestep)

width  = camera.getWidth()
height = camera.getHeight()

# =====================================================
# DEPTH CAMERA (range finder from Astra RGB-D)
# =====================================================

range_finder = robot.getDevice('camera depth')
range_finder.enable(timestep)

# =====================================================
# LIDAR
# =====================================================

lidar = robot.getDevice('laser')
lidar.enable(timestep)

# =====================================================
# CONSTANTS
# =====================================================

WHEEL_RADIUS       = 0.043   # metres
TRACK_WIDTH        = 0.220   # metres
MAX_SPEED          = 5.0
OBSTACLE_THRESHOLD = 0.35    # metres — LIDAR trigger distance
GOAL_DISTANCE      = 0.10    # metres — proximity threshold for goal
SENSOR_X_OFFSET    = 0.1     # metres — sensors sit 0.1m ahead of robot centre
CAMERA_X_OFFSET    = 0.027   # metres — depth camera sits 0.027m behind robot centre
BOX_HALF_SIZE      = 0.085   # metres — half of 0.17m target box
GOAL_RED_THRESHOLD = (width // 10) * (height // 10) // 5  # 20% of sampled pixels

# =====================================================
# LIDAR DISTANCES
# =====================================================

def get_lidar_distances():
    range_image = lidar.getRangeImage()
    n = len(range_image)
    arc = n // 12  # 30 degrees of points

    def sector_min(center, half_width):
        vals = []
        for i in range(center - half_width, center + half_width):
            v = range_image[i % n]
            if not math.isfinite(v):
                continue
            vals.append(v if v > 0 else 0.01)  # 0 = below min range, treat as very close
        return min(vals) if vals else float('inf')

    front = sector_min(n // 2,     arc)  # forward
    left  = sector_min(n // 4,     arc)  # left
    right = sector_min(3 * n // 4, arc)  # right
    return front, left, right

# =====================================================
# MOVEMENT FUNCTIONS
# =====================================================

def forward(speed=4.0):
    fl.setVelocity(speed)
    fr.setVelocity(speed)
    rl.setVelocity(speed)
    rr.setVelocity(speed)

def turn_left():
    fl.setVelocity(-3.5)
    fr.setVelocity(3.5)
    rl.setVelocity(-3.5)
    rr.setVelocity(3.5)

def turn_right():
    fl.setVelocity(3.5)
    fr.setVelocity(-3.5)
    rl.setVelocity(3.5)
    rr.setVelocity(-3.5)

def stop():
    for m in motors:
        m.setVelocity(0.0)

# =====================================================
# TARGET DETECTION
# =====================================================

def detect_target():
    image = camera.getImage()
    if image is None:
        return False, 0, 0

    red_pixels = 0
    target_x   = 0

    for x in range(0, width, 10):
        for y in range(0, height, 10):
            r = camera.imageGetRed(image, width, x, y)
            g = camera.imageGetGreen(image, width, x, y)
            b = camera.imageGetBlue(image, width, x, y)

            if r > 150 and g < 80 and b < 80:
                red_pixels += 1
                target_x   += x

    if red_pixels > 5:
        return True, target_x / red_pixels, red_pixels

    return False, 0, 0
