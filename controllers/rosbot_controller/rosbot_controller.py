from controller import Robot, GPS, Compass
import math
import random
import time as tm
import json

robot = Robot()
timestep = int(robot.getBasicTimeStep())
dt = timestep / 1000.0

# Devices
gps = robot.getDevice("gps")
compass = robot.getDevice("compass")
gps.enable(timestep)
compass.enable(timestep)


motors = [robot.getDevice(name) for name in [
    "fl_wheel_joint", "fr_wheel_joint", "rl_wheel_joint", "rr_wheel_joint"
]]
for m in motors:
    m.setPosition(float('inf'))

def set_velocity(left, right):
    motors[0].setVelocity(left)
    motors[2].setVelocity(left)
    motors[1].setVelocity(right)
    motors[3].setVelocity(right)

# Constants
MAX_SPEED = 6.0
ARENA_RADIUS = 2.5
WALL_TOLERANCE = 0.1
CENTER_TOLERANCE = 0.1
BACK_DURATION = 1.0  # seconds to back up before turning
center_x, center_y = 0.0, 0.0


# State machine
state = "choose_direction"
back_start_time = None

current_left_speed = 0.0
current_right_speed = 0.0
has_collected = False   # Flag: one collection per trip
t0 = True
collected_count = 0
        
def get_heading_angle():
    north = compass.getValues()
    # Heading is perpendicular to the compass vector (which points to global north in robot's frame)
    # So robot heading in world frame is: atan2(-north[0], -north[1])
    heading_angle = math.atan2(-north[0], -north[1])
    return heading_angle
    
def get_heading_vector():
    angle = get_heading_angle()
    #print(f"[Heading] Angle: {angle:.4f} rad ({math.degrees(angle):.2f}°)")
    return [math.cos(angle), math.sin(angle)]
    
def angle_to_center(x, y, cx, cy):
    dx = cx - x
    dy = cy - y
    return math.atan2(dy, dx)  # Desired heading to center

def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle
    
def will_hit_center(robot_x, robot_y, heading, center_x, center_y):

    dx, dy = heading
    A = dy
    B = -dx
    C = dx * robot_y - dy * robot_x

    numerator = abs(A * center_x + B * center_y + C)
    denominator = math.sqrt(A**2 + B**2)
    distance = numerator / denominator if denominator != 0 else float('inf')

    # Direction check via dot product
    to_center_x = center_x - robot_x
    to_center_y = center_y - robot_y
    dot_product = dx * to_center_x + dy * to_center_y

    return distance < CENTER_TOLERANCE and dot_product < 0


while robot.step(timestep) != -1:
    pos = gps.getValues()
    x, y = pos[0], pos[1]
    
    dist = math.sqrt((x-center_x)**2 + (y-center_y)**2)
    time = robot.getTime()
    
    get_heading_vector()
    
    if state == "choose_direction":
        print("Choosing random orientation.")
        target_heading = random.uniform(-math.pi, math.pi)  # random angle in radians
        state = "orienting"
    
    elif state == "orienting":
        current_heading = get_heading_angle()
        error = normalize_angle(target_heading - current_heading)
    
        if abs(error) < math.radians(2):  # aligned within 5 degrees
            print("Oriented! Moving out.")
            set_velocity(MAX_SPEED, MAX_SPEED)
            state = "moving_out"
        else:
            turn_speed = MAX_SPEED * 0.5
            if error > 0:
                set_velocity(-turn_speed, turn_speed)
            else:
                set_velocity(turn_speed, -turn_speed)
            
    elif state == "moving_out":
        set_velocity(MAX_SPEED, MAX_SPEED)
        if dist >= ARENA_RADIUS - WALL_TOLERANCE:
            print("Wall reached. Backing up.")
            back_start_time = time
            set_velocity(-MAX_SPEED, -MAX_SPEED)
            state = "backing_up"

        has_collected = robot.getCustomData().strip().lower() == "collected"
        if has_collected:
            print(f"Resource detected! Collecting...")
            set_velocity(0, 0)
            start = robot.getTime()
            while robot.step(timestep) != -1:
                if robot.getTime() - start > 2.0:
                    break  # Simulated recollection

            print(f"Collected 1 unit from a resource, returning to nest")
            state = "turning_to_center"

    elif state == "backing_up":
        set_velocity(-MAX_SPEED*0.5, -MAX_SPEED*0.5)
        if time - back_start_time >= BACK_DURATION:
            state = "turning_to_center"

    elif state == "turning_to_center":
        current_left_speed = -MAX_SPEED
        current_right_speed = MAX_SPEED
        set_velocity(current_left_speed, current_right_speed)
        
        # Use heading to check alignment
        if will_hit_center(x, y, get_heading_vector(), center_x, center_y):
            print("Aligned with center. Returning.")
            set_velocity(MAX_SPEED, MAX_SPEED)
            state = "returning"
        else:
            set_velocity(-MAX_SPEED * 0.5, MAX_SPEED * 0.5)  # keep turning

    elif state == "returning":
            
        if dist <= CENTER_TOLERANCE + 0.2:
            print("Returned to center.")
            set_velocity(0, 0)

            start = robot.getTime()
            while robot.step(timestep) != -1:
                
                has_collected = robot.getCustomData().strip().lower() == "collected"
                
                if (robot.getTime() - start > 2.0) and not has_collected:
                    break

            #if has_collected:
            #    raise RuntimeError("Robot has already collected resources in this trip " \
            #    "but its arguments were not updated correctly.")
                
            state = "choose_direction"
                

            
