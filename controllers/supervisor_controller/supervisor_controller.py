from controller import Supervisor
from collections import Counter
import random
import math
import json
import os

supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())

# Constants
RADIUS = 2.5
NUM_RESOURCES = 10
RESOURCE_MIN = 1
RESOURCE_MAX = 10

CENTER_TOLERANCE = 0.1

resource_data = []
robot_teams = {}
robot_names = []
has_collected_states = {}

root = supervisor.getRoot()
children = root.getField("children")
count = children.getCount()

for i in range(count):
    node = children.getMFNode(i)
    if node.getTypeName() == "Rosbot":
        name = node.getField("name").getSFString()
        robot_names.append(name)
        team_data = node.getField("controllerArgs").getMFString(0)
        team = team_data.strip().lower()
        robot_teams[name] = team

for robot in robot_names:
    has_collected_states[robot] = 'not_collected'

def generate_resource_spot(supervisor, value, index):
    angle = random.uniform(0, 2 * math.pi)
    distance = random.uniform(1.0, RADIUS - 0.2)
    x = distance * math.cos(angle)
    y = distance * math.sin(angle)
    radius = 0.1 + (value - 1) * 0.02  # Size proportional to value
    resource_data.append({
        "name": f"resource_{index}",
        "x": x,
        "y": y,
        "value": value
    })

    root = supervisor.getRoot()
    children_field = root.getField("children")
    children_field.importMFNodeFromString(-1, f'''
    DEF RESOURCE_{index}
    Solid {{
      translation {x} {y} 0.01
      rotation 0 0 1 0
      name "resource_{index}"
      children [
        Shape {{
          appearance PBRAppearance {{
            baseColor 1 1 0
            roughness 1
          }}
          geometry Cylinder {{
            radius {radius}
            height 0.02
          }}
        }}
      ]
    }}
    ''')


def generate_resources(supervisor):
    for i in range(NUM_RESOURCES):
        value = random.randint(RESOURCE_MIN, RESOURCE_MAX)
        generate_resource_spot(supervisor, value, i)

# Only generate resources once at simulation start
generate_resources(supervisor)

with open("./../../data/resource_positions.json", "w") as f:
    json.dump(resource_data, f)

RESOURCE_JSON = "./../../data/resource_positions.json"
LOG_FILE = "./../../data/recollection_log.txt"

def update_resource_shapes():
    for i, resource in enumerate(resource_data):
        value = resource["value"]
        node = supervisor.getFromDef(f"RESOURCE_{i}")

        if not node:
            continue  # Skip if the node doesn't exist

        if value <= 0:
            # Remove the node from the world
            node.remove()
            print(f"Resource {i} depleted and removed.")
        else:
            # Update the radius
            radius = 0.1 + (value - 1) * 0.02
            children_field = node.getField("children")
            shape_node = children_field.getMFNode(0)
            geometry_field = shape_node.getField("geometry")
            cylinder_node = geometry_field.getSFNode()
            radius_field = cylinder_node.getField("radius")
            radius_field.setSFFloat(radius)

def check_and_process_collection(robot_name):
    robot_node = supervisor.getFromDef(robot_name)
    position = robot_node.getField("translation").getSFVec3f()
    x, y = position[0], position[1]
    for i, resource in enumerate(resource_data):
        value = resource["value"]
        if value <= 0:
            continue
        rx, ry = resource["x"], resource["y"]

        detection_radius = 0.2 + (value - 1) * 0.02
        dist = math.sqrt((x - rx)**2 + (y - ry)**2)
        if (dist <= detection_radius) and has_collected_states[robot_name] == 'not_collected':
            has_collected_states[robot_name] = 'collected'
            resource["value"] = max(0, value - 1)
            robot_node.getField("customData").setSFString("collected")
            print(f"[Supervisor] {robot_name} collected from resource_{i}, remaining value: {resource['value']}")
            return True

def track_resources_collection():
    for robot_name in robot_names:
        check_and_process_collection(robot_name)

def has_recollected(rosbot_name):
    robot_node = supervisor.getFromDef(rosbot_name)
    if robot_node is None:
        raise ValueError(f"Robot {rosbot_name} not found in the simulation.")
    customdata_field_string = robot_node.getField("customData").getSFString()
    return customdata_field_string == "collected"

def log_collected_resources():

    for robot_name in robot_names:
        position = supervisor.getFromDef(robot_name).getField("translation").getSFVec3f()
        distance = math.sqrt(position[0]**2 + position[1]**2)
        #print(f"{robot_name} position: {position}, distance from center: {distance:.2f}")

        if distance <= CENTER_TOLERANCE + 0.2:
            if not has_recollected(robot_name):
                continue
            else:
                robot_node = supervisor.getFromDef(robot_name)
                if robot_node:
                    customdata_field = robot_node.getField("customData")
                    customdata_field.setSFString("not_collected")
                    has_collected_states[robot_name] = 'not_collected'
                    print(f"{robot_name}'s status is now 'not_collected'.")

                with open(LOG_FILE, "a") as f:
                    team = robot_teams.get(robot_name, "unknown")
                    teams_resources_counter[team] += 1
                    log_entry = {
                        "robot": robot_name,
                        "team": team,
                        "resources_collected": teams_resources_counter[team]
                    }
                    f.write(json.dumps(log_entry) + "\n")

teams_resources_counter = Counter()
# Remove existing log file if it exists
if os.path.exists(LOG_FILE):
    os.remove(LOG_FILE)
# Main loop just steps the simulation
while supervisor.step(timestep) != -1:

    track_resources_collection()

    update_resource_shapes()

    log_collected_resources()
    
        
