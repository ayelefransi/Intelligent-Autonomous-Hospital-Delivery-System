#!/usr/bin/env python3
"""
add_dynamic_actors.py — Inject moving people into the hospital world.

Adds walking doctors, patients, and visitors as Gazebo <actor> elements
with predefined trajectories through the hospital corridors.

Usage: python3 add_dynamic_actors.py <path_to_hospital.world>
"""

import sys
import os
import re

ACTOR_TEMPLATES = [
    # doctor_1 — walks between nurse station and pharmacy
    {
        'name': 'doctor_1',
        'skin': 'https://fuel.gazebosim.org/1.0/OpenRobotics/models/actor_dummy/tip/files/meshes/walking.dae',
        'animation': 'https://fuel.gazebosim.org/1.0/OpenRobotics/models/actor_dummy/tip/files/meshes/walking.dae',
        'pose': '0 1.5 1.01 0 0 0',
        'waypoints': [
            (0,   '0 1.5 1.01 0 0 0'),
            (15,  '9 10 1.01 0 0 0'),    # walk to pharmacy
            (35,  '9 10 1.01 0 0 0'),    # wait at pharmacy
            (50,  '0 1.5 1.01 0 0 0'),   # return to nurse station
        ],
    },
    # doctor_2 — patrols patient room hallway
    {
        'name': 'doctor_2',
        'skin': 'https://fuel.gazebosim.org/1.0/OpenRobotics/models/actor_dummy/tip/files/meshes/walking.dae',
        'animation': 'https://fuel.gazebosim.org/1.0/OpenRobotics/models/actor_dummy/tip/files/meshes/walking.dae',
        'pose': '11 -2 1.01 0 0 1.57',
        'waypoints': [
            (0,   '11 -2 1.01 0 0 1.57'),
            (12,  '11 -7 1.01 0 0 1.57'),   # walk to room2
            (30,  '11 -7 1.01 0 0 1.57'),   # wait
            (42,  '11 -18 1.01 0 0 1.57'),  # walk to room3
            (60,  '11 -18 1.01 0 0 1.57'),  # wait
            (72,  '11 -2 1.01 0 0 1.57'),   # return
        ],
    },
    # visitor_1 — walks from reception to patient_room4
    {
        'name': 'visitor_1',
        'skin': 'https://fuel.gazebosim.org/1.0/OpenRobotics/models/actor_dummy/tip/files/meshes/walking.dae',
        'animation': 'https://fuel.gazebosim.org/1.0/OpenRobotics/models/actor_dummy/tip/files/meshes/walking.dae',
        'pose': '0 -5.5 1.01 0 0 0',
        'waypoints': [
            (0,   '0 -5.5 1.01 0 0 0'),
            (18,  '-11 0 1.01 0 0 3.14'),  # walk to room4
            (38,  '-11 0 1.01 0 0 3.14'),  # wait
            (56,  '0 -5.5 1.01 0 0 0'),    # return to reception
        ],
    },
    # patient_1 — wheelchair being pushed from room1 to lab
    {
        'name': 'patient_1',
        'skin': 'https://fuel.gazebosim.org/1.0/OpenRobotics/models/actor_dummy/tip/files/meshes/walking.dae',
        'animation': 'https://fuel.gazebosim.org/1.0/OpenRobotics/models/actor_dummy/tip/files/meshes/walking.dae',
        'pose': '11 -2 1.01 0 0 0',
        'waypoints': [
            (0,   '11 -2 1.01 0 0 0'),
            (20,  '-1 -21 1.01 0 0 0'),     # to lab
            (50,  '-1 -21 1.01 0 0 0'),     # at lab
            (70,  '11 -2 1.01 0 0 0'),      # return
        ],
    },
    # nurse_1 — moves between nurse station and supply_room (unpredictable)
    {
        'name': 'nurse_1',
        'skin': 'https://fuel.gazebosim.org/1.0/OpenRobotics/models/actor_dummy/tip/files/meshes/walking.dae',
        'animation': 'https://fuel.gazebosim.org/1.0/OpenRobotics/models/actor_dummy/tip/files/meshes/walking.dae',
        'pose': '0 1.5 1.01 0 0 0',
        'waypoints': [
            (0,   '0 1.5 1.01 0 0 0'),
            (10,  '-10 10 1.01 0 0 3.14'),  # to supply room
            (25,  '-10 10 1.01 0 0 3.14'),  # load supplies
            (30,  '0 1.5 1.01 0 0 0'),      # return (quick)
            (40,  '9 10 1.01 0 0 0'),       # to pharmacy
            (55,  '9 10 1.01 0 0 0'),       # at pharmacy
            (65,  '0 1.5 1.01 0 0 0'),      # return
        ],
    },
]


def build_actor_xml(actor):
    """Build a Gazebo <actor> XML block."""
    wp_xml = '\n'.join(
        f'            <waypoint><time>{t}</time><pose>{p}</pose></waypoint>'
        for t, p in actor['waypoints']
    )
    return f'''    <actor name="{actor['name']}">
      <pose>{actor['pose']}</pose>
      <skin><filename>{actor['skin']}</filename></skin>
      <animation name="walking"><filename>{actor['animation']}</filename></animation>
      <script>
        <loop>true</loop>
        <trajectory id="0" type="walking">
{wp_xml}
        </trajectory>
      </script>
    </actor>
'''


def inject_actors(world_path):
    """Read the world file, inject actors before </world>, write back."""
    with open(world_path, 'r') as f:
        content = f.read()

    # Build all actor XML blocks
    actors_xml = '\n'.join(build_actor_xml(a) for a in ACTOR_TEMPLATES)

    # Inject before closing </world> tag
    if '</world>' not in content:
        print(f"ERROR: No </world> tag found in {world_path}")
        return

    content = content.replace('</world>', actors_xml + '\n</world>')

    with open(world_path, 'w') as f:
        f.write(content)

    print(f"Injected {len(ACTOR_TEMPLATES)} dynamic actors into {world_path}")


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <path_to_hospital.world>")
        sys.exit(1)

    world_path = sys.argv[1]
    if not os.path.exists(world_path):
        print(f"ERROR: File not found: {world_path}")
        sys.exit(1)

    inject_actors(world_path)
