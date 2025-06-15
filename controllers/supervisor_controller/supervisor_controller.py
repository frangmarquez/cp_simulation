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

def get_team_from_color(rgb, tolerance=0.1):
    r, g, b = rgb
    if r >= g + tolerance and r >= b + tolerance:
        return "red"
    elif g >= r + tolerance and g >= b + tolerance:
        return "green"
    elif b >= r + tolerance and b >= g + tolerance:
        return "blue"
    else:
        return "unknown"

def read_team_scores():
    scores = Counter()
    if not os.path.exists(LOG_FILE):
        return scores

    with open(LOG_FILE, "r") as f:
        for line in f:
            try:
                entry = json.loads(line.strip())
                team = entry.get("team", "").lower()
                if team:
                    scores[team] += 1
            except json.JSONDecodeError:
                continue  # skip malformed lines
    return scores

def print_scoreboard_if_changed(current_scores):
    global last_printed_scores
    if current_scores != last_printed_scores:
        print("\n" + "="*32)
        print(f"{'TEAM':<15} | {'RESOURCES':>10}")
        print("-"*32)
        for team, score in sorted(current_scores.items()):
            print(f"{team.capitalize():<15} | {score:>10}")
        print("="*32 + "\n")
        last_printed_scores = current_scores.copy()
    
robot_teams = {}
robot_names = []
has_collected_states = {}
for robot in robot_names:
    has_collected_states[robot] = 'not_collected'

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
LOG_FILE = "./../../data/recollection_log.json"
last_printed_scores = Counter()


def update_resource_shapes():
    try:
        with open(RESOURCE_JSON, "r") as f:
            updated_resources = json.load(f)

        for i, resource in enumerate(updated_resources):
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

    except Exception as e:
        print("Error updating resources:", e)

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

    update_resource_shapes()

    log_collected_resources()
    
        
