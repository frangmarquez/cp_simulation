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
NUM_RESOURCES = 15
RESOURCE_MIN = 1
RESOURCE_MAX = 10

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

root = supervisor.getRoot()
children = root.getField("children")
count = children.getCount()

for i in range(count):
    node = children.getMFNode(i)
    if node.getTypeName() == "Robot":
        name = node.getField("name").getSFString()
        if name.startswith("rosbot"):
            custom_data = node.getField("customData").getSFString()
            team = custom_data.strip().lower()
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

        
# Main loop just steps the simulation
while supervisor.step(timestep) != -1:
    #current_scores = read_team_scores()
    #print_scoreboard_if_changed(current_scores)
    update_resource_shapes()
    

