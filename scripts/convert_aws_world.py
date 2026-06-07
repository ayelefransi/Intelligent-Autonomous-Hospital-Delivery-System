#!/usr/bin/env python3
"""
convert_aws_world.py — Convert AWS RoboMaker Hospital World to Gazebo Harmonic.
- SDF 1.6 → 1.9
- Rename world "world" → "hospital"
- Remove sun/ground_plane
- Add system plugins
- Strip unresolvable fuel models (keep only AWS hospital models)
"""
import os, re, sys

WORLD_DIR = sys.argv[1] if len(sys.argv) > 1 else "."

for world_filename in ["hospital.world", "hospital_two_floors.world", "hospital_three_floors.world"]:
    world_file = os.path.join(WORLD_DIR, world_filename)
    if not os.path.isfile(world_file):
        print(f"  SKIP: {world_filename} not found")
        continue

    print(f"=== Converting {world_filename} for Gazebo Harmonic ===")
    with open(world_file, 'r') as f:
        content = f.read()

    # 1. SDF version
    content = content.replace('sdf version="1.6"', 'sdf version="1.9"')

    # 2. World name
    content = content.replace('world name="world"', 'world name="hospital"')

    # 3. Remove sun and ground_plane
    content = re.sub(r'\s*<include>\s*<uri>model://sun</uri>\s*</include>\s*', '\n', content)
    content = re.sub(r'\s*<include>\s*<uri>model://ground_plane</uri>.*?</include>\s*', '\n', content, flags=re.DOTALL)

    # 4. Add system plugins after world opening tag
    plugins = """    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>\n"""
    content = content.replace('<world name="hospital">', '<world name="hospital">\n' + plugins)

    # 5. Add aggressive lighting (fix dark rendering on llvmpipe/software renderer)
    # Remove old <scene> block
    content = re.sub(r'\s*<scene>.*?</scene>\s*', '\n', content, flags=re.DOTALL)
    # Add lighting block after gravity
    lighting = """    <scene>
      <ambient>1.0 1.0 1.0 1.0</ambient>
      <background>0.7 0.7 0.7 1.0</background>
      <shadows>false</shadows>
    </scene>
    <light type="directional" name="sun_light">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 30 0 0 0</pose>
      <diffuse>1.0 1.0 1.0 1.0</diffuse>
      <specular>0.8 0.8 0.8 1.0</specular>
      <direction>0 0 -1</direction>
    </light>
    <light type="point" name="ceiling_light_1">
      <pose>-5 -5 5 0 0 0</pose>
      <diffuse>1.0 1.0 1.0 1.0</diffuse>
      <specular>0.5 0.5 0.5 1.0</specular>
      <attenuation><range>40</range></attenuation>
    </light>
    <light type="point" name="ceiling_light_2">
      <pose>5 -5 5 0 0 0</pose>
      <diffuse>1.0 1.0 1.0 1.0</diffuse>
      <specular>0.5 0.5 0.5 1.0</specular>
      <attenuation><range>40</range></attenuation>
    </light>
    <light type="point" name="ceiling_light_3">
      <pose>0 5 5 0 0 0</pose>
      <diffuse>1.0 1.0 1.0 1.0</diffuse>
      <specular>0.5 0.5 0.5 1.0</specular>
      <attenuation><range>40</range></attenuation>
    </light>
    <light type="point" name="ceiling_light_4">
      <pose>0 -15 5 0 0 0</pose>
      <diffuse>1.0 1.0 1.0 1.0</diffuse>
      <specular>0.5 0.5 0.5 1.0</specular>
      <attenuation><range>40</range></attenuation>
    </light>
    <light type="point" name="ceiling_light_5">
      <pose>-10 5 5 0 0 0</pose>
      <diffuse>1.0 1.0 1.0 1.0</diffuse>
      <specular>0.5 0.5 0.5 1.0</specular>
      <attenuation><range>40</range></attenuation>
    </light>
    <light type="point" name="ceiling_light_6">
      <pose>10 5 5 0 0 0</pose>
      <diffuse>1.0 1.0 1.0 1.0</diffuse>
      <specular>0.5 0.5 0.5 1.0</specular>
      <attenuation><range>40</range></attenuation>
    </light>\n"""
    content = content.replace('<gravity>', lighting + '\n    <gravity>')

    # 6. Strip unresolvable fuel models — remove <model> blocks whose URI isn't aws_robomaker_*
    def strip_unresolvable_models(text):
        """Remove any <model> block where the model:// URI is NOT an AWS model."""
        # Find all <model...>...</model> blocks
        model_pattern = re.compile(r'(<model\s[^>]*>.*?</model>)', re.DOTALL)
        uri_pattern = re.compile(r'<uri>\s*model://([^/\s<]+)\s*</uri>')

        def should_keep(match):
            block = match.group(1)
            uri_match = uri_pattern.search(block)
            if not uri_match:
                return block  # No URI? Keep it
            model_name = uri_match.group(1)
            if 'aws_robomaker' in model_name:
                return block  # AWS model, keep
            print(f"  STRIP: {model_name}")
            return ''  # Remove unresolvable fuel model

        return model_pattern.sub(should_keep, text)

    content = strip_unresolvable_models(content)

    with open(world_file, 'w') as f:
        f.write(content)

print("Done converting AWS worlds.")
