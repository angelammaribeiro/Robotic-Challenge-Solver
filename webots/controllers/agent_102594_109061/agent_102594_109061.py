""""my_controller controller."""

from controller import Robot, Camera
import logging

logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s [%(levelname)s] %(message)s")

robot = Robot()
timeStep = int(robot.getBasicTimeStep())

# ===== Constants (your logic unchanged) =====
MAX_SPEED = 6.28
k = 0.003

# ===== Motors =====
leftMotor = robot.getDevice("left wheel motor")
rightMotor = robot.getDevice("right wheel motor")
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# ===== Sensors =====
ps_names = ['ps0','ps1','ps2','ps3','ps4','ps5','ps6','ps7']
ps = []
for name in ps_names:
    s = robot.getDevice(name)
    s.enable(timeStep)
    ps.append(s)

# ===== Camera for checkpoints =====
cam = robot.getDevice("camera")
cam.enable(timeStep)

def detect_color():
    """Simple average RGB from the whole image -> 'red','blue','yellow','none'."""
    img = cam.getImage()
    if img is None: return 'none'
    w, h = cam.getWidth(), cam.getHeight()
    if w == 0 or h == 0: return 'none'
    r = g = b = 0
    for y in range(h):
        for x in range(w):
            r += Camera.imageGetRed(img,  w, x, y)
            g += Camera.imageGetGreen(img,w, x, y)
            b += Camera.imageGetBlue(img, w, x, y)
    n = w*h
    r /= n; g /= n; b /= n
    if r > 120 and r > g*1.3 and r > b*1.3: return "red"
    if b > 120 and b > r*1.3 and b > g*1.3: return "blue"
    if r > 100 and g > 100 and b < 80: return "yellow"
    return "none"

# ===== Checkpoint order =====
ORDER = ["red", "blue", "yellow"]
last_index = -1   # start before any color
lap_count = 0     # counts completed laps

while robot.step(timeStep) != -1:
    v = 0.4 * MAX_SPEED
    vals = [s.getValue() for s in ps]
    sim_time = robot.getTime()

    # === Your obstacle avoidance ===
    front_right = vals[1]
    side_right  = vals[2]
    side_left   = vals[5]
    front_left  = vals[6]
    d_left  = front_left + side_left 
    d_right = front_right + side_right
    delta_d = d_left - d_right

    delta_v = k * delta_d
    right_speed = v - delta_v
    left_speed  = v + delta_v

    left_speed  = max(-MAX_SPEED, min(MAX_SPEED, left_speed))
    right_speed = max(-MAX_SPEED, min(MAX_SPEED, right_speed))
    leftMotor.setVelocity(left_speed)
    rightMotor.setVelocity(right_speed)

    # === Checkpoint detection ===
    color = detect_color()
    if color in ORDER:
        idx = ORDER.index(color)
        if idx == last_index:
            pass  # still on the same checkpoint
        elif idx == (last_index + 1) % len(ORDER):   # ✅ loop order
            # Check if we wrapped from yellow -> red
            if last_index == len(ORDER) - 1 and idx == 0:
                lap_count += 1
                logging.info(f"🏁 Lap completed! Total laps: {lap_count}")
            last_index = idx
            logging.info(f"✅ Passed checkpoint: {color}")
        else:
            expected = ORDER[(last_index + 1) % len(ORDER)]
            logging.warning(f"❌ Wrong checkpoint order! Saw {color}, expected {expected}")

    logging.info(f"t={sim_time:.2f}s color={color} lap={lap_count} "
                 f"cmd L={left_speed:.2f} R={right_speed:.2f}")
