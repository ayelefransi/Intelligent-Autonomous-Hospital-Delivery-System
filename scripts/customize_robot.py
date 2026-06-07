#!/usr/bin/env python3
"""
customize_robot.py
==================
Modifies the TurtleBot3 Waffle SDF model and URDF:
  1. Scale robot BODY visuals non-uniformly (2x width, 5x height)
  2. Scale WHEEL visuals uniformly (keep circular!)
  3. Scale robot PHYSICS uniformly (2x) so diff_drive works
  4. Update diff_drive plugin wheel params
  5. Change all colors to completely white

Usage:
    python3 customize_robot.py /path/to/turtlebot3_gazebo
"""

import sys
import os
import re
import xml.etree.ElementTree as ET

# ── Scale factors ────────────────────────────────────────────────────────
SCALE_XY = 2.0          # Uniform physics scale (width & depth)
SCALE_Z_VISUAL = 5.0    # Visual-only height stretch (body only)
SCALE_Z_PHYSICS = 2.0   # Physics height (must match XY for diff_drive)
S = SCALE_XY            # Shorthand for uniform physics scale

# Wheel link names in TurtleBot3 Waffle SDF
WHEEL_KEYWORDS = ['wheel', 'caster']


def _is_wheel_link(element):
    """Check if an element or any of its parents is a wheel link."""
    # Walk up the tree checking link names
    name = element.get('name', '')
    if any(kw in name.lower() for kw in WHEEL_KEYWORDS):
        return True
    return False


def _find_parent_link_name(root, element):
    """Find the parent link name of a given element by searching the tree."""
    for link in root.iter('link'):
        link_name = link.get('name', '')
        for child in link.iter():
            if child is element:
                return link_name
    return ''


def scale_sdf(sdf_path: str):
    """Scale SDF: body visuals non-uniform, wheels uniform, physics uniform."""
    tree = ET.parse(sdf_path)
    root = tree.getroot()

    # Build a map of which links are wheel links
    wheel_links = set()
    for link in root.iter('link'):
        link_name = link.get('name', '')
        if any(kw in link_name.lower() for kw in WHEEL_KEYWORDS):
            wheel_links.add(link_name)

    # ── 1. Scale VISUAL geometry ────────────────────────────────────────
    for link in root.iter('link'):
        link_name = link.get('name', '')
        is_wheel = link_name in wheel_links

        for visual in link.iter('visual'):
            if is_wheel:
                # Wheels: scale uniformly to stay CIRCULAR
                for mesh in visual.iter('mesh'):
                    scale_el = mesh.find('scale')
                    if scale_el is not None and scale_el.text:
                        vals = scale_el.text.strip().split()
                        new_vals = [str(float(v) * S) for v in vals]
                        scale_el.text = ' '.join(new_vals)
                    else:
                        scale_el = ET.SubElement(mesh, 'scale')
                        scale_el.text = f'{S} {S} {S}'

                for cylinder in visual.iter('cylinder'):
                    r = cylinder.find('radius')
                    if r is not None and r.text:
                        r.text = str(float(r.text) * S)
                    l = cylinder.find('length')
                    if l is not None and l.text:
                        l.text = str(float(l.text) * S)

                for sphere in visual.iter('sphere'):
                    r = sphere.find('radius')
                    if r is not None and r.text:
                        r.text = str(float(r.text) * S)
            else:
                # Body: scale non-uniformly (2x wide, 5x tall)
                for mesh in visual.iter('mesh'):
                    scale_el = mesh.find('scale')
                    if scale_el is not None and scale_el.text:
                        vals = scale_el.text.strip().split()
                        new_vals = [
                            str(float(vals[0]) * S),
                            str(float(vals[1]) * S),
                            str(float(vals[2]) * SCALE_Z_VISUAL),
                        ]
                        scale_el.text = ' '.join(new_vals)
                    else:
                        scale_el = ET.SubElement(mesh, 'scale')
                        scale_el.text = f'{S} {S} {SCALE_Z_VISUAL}'

                for cylinder in visual.iter('cylinder'):
                    r = cylinder.find('radius')
                    if r is not None and r.text:
                        r.text = str(float(r.text) * S)
                    l = cylinder.find('length')
                    if l is not None and l.text:
                        l.text = str(float(l.text) * SCALE_Z_VISUAL)

                for box in visual.iter('box'):
                    size_el = box.find('size')
                    if size_el is not None and size_el.text:
                        vals = size_el.text.strip().split()
                        new_vals = [
                            str(float(vals[0]) * S),
                            str(float(vals[1]) * S),
                            str(float(vals[2]) * SCALE_Z_VISUAL),
                        ]
                        size_el.text = ' '.join(new_vals)

    # ── 2. Scale COLLISION geometry uniformly (2x, 2x, 2x) ─────────────
    for collision in root.iter('collision'):
        for mesh in collision.iter('mesh'):
            scale_el = mesh.find('scale')
            if scale_el is not None and scale_el.text:
                vals = scale_el.text.strip().split()
                new_vals = [str(float(v) * S) for v in vals]
                scale_el.text = ' '.join(new_vals)
            else:
                scale_el = ET.SubElement(mesh, 'scale')
                scale_el.text = f'{S} {S} {S}'

        for cylinder in collision.iter('cylinder'):
            r = cylinder.find('radius')
            if r is not None and r.text:
                r.text = str(float(r.text) * S)
            l = cylinder.find('length')
            if l is not None and l.text:
                l.text = str(float(l.text) * S)

        for box in collision.iter('box'):
            size_el = box.find('size')
            if size_el is not None and size_el.text:
                vals = size_el.text.strip().split()
                new_vals = [str(float(v) * S) for v in vals]
                size_el.text = ' '.join(new_vals)

        for sphere in collision.iter('sphere'):
            r = sphere.find('radius')
            if r is not None and r.text:
                r.text = str(float(r.text) * S)

    # ── 3. Scale ALL poses uniformly (keeps wheels on ground) ───────────
    for pose in root.iter('pose'):
        if pose.text:
            vals = pose.text.strip().split()
            if len(vals) >= 3:
                new_vals = [
                    str(float(vals[0]) * S),
                    str(float(vals[1]) * S),
                    str(float(vals[2]) * S),
                ]
                new_vals += vals[3:]  # rotation stays the same
                pose.text = ' '.join(new_vals)

    # ── 4. Update diff_drive plugin parameters ──────────────────────────
    for plugin in root.iter('plugin'):
        for ws in plugin.iter('wheel_separation'):
            if ws.text:
                ws.text = str(float(ws.text) * S)
        for wr in plugin.iter('wheel_radius'):
            if wr.text:
                wr.text = str(float(wr.text) * S)

    # ── 5. Change all colors to white ───────────────────────────────────
    for ambient in root.iter('ambient'):
        ambient.text = '1.0 1.0 1.0 1.0'
    for diffuse in root.iter('diffuse'):
        diffuse.text = '1.0 1.0 1.0 1.0'
    for specular in root.iter('specular'):
        specular.text = '1.0 1.0 1.0 1.0'
    for emissive in root.iter('emissive'):
        emissive.text = '0.3 0.3 0.3 1.0'

    # ── 6. Scale LiDAR ranges (push min_range outside chassis) ──────────
    for sensor_tag in ['ray', 'lidar']:
        for sensor in root.iter(sensor_tag):
            for range_el in sensor.iter('range'):
                min_r = range_el.find('min')
                max_r = range_el.find('max')
                if min_r is not None and min_r.text:
                    chassis_clearance = 0.14 * S + 0.05
                    min_r.text = str(max(float(min_r.text) * S, chassis_clearance))
                if max_r is not None and max_r.text:
                    max_r.text = str(float(max_r.text) * S)

    tree.write(sdf_path, xml_declaration=True, encoding='unicode')
    print(f'  [SDF] Body: {S}x/{S}x/{SCALE_Z_VISUAL}x  Wheels: {S}x uniform (circular)  Physics: {S}x uniform  White: {sdf_path}')


def scale_urdf(urdf_path: str):
    """Scale URDF geometry uniformly for TF frames."""
    with open(urdf_path, 'r') as f:
        content = f.read()

    def _safe_scale_xyz(match):
        prefix = match.group(1)
        vals_str = match.group(2).strip()
        if '$' in vals_str or '{' in vals_str:
            return match.group(0)
        vals = vals_str.split()
        if len(vals) >= 3:
            try:
                nv = [str(float(v) * S) for v in vals]
                return f'{prefix}xyz="{" ".join(nv)}"'
            except ValueError:
                pass
        return match.group(0)

    def _safe_scale_scale(match):
        vals_str = match.group(1).strip()
        if '$' in vals_str or '{' in vals_str:
            return match.group(0)
        vals = vals_str.split()
        if len(vals) >= 3:
            try:
                nv = [str(float(v) * S) for v in vals]
                return f'scale="{" ".join(nv)}"'
            except ValueError:
                pass
        return match.group(0)

    def _safe_scale_size(match):
        vals_str = match.group(1).strip()
        if '$' in vals_str or '{' in vals_str:
            return match.group(0)
        vals = vals_str.split()
        if len(vals) >= 3:
            try:
                nv = [str(float(v) * S) for v in vals]
                return f'size="{" ".join(nv)}"'
            except ValueError:
                pass
        return match.group(0)

    def _safe_scale_single(attr_name):
        def replacer(match):
            v = match.group(1).strip()
            if '$' in v or '{' in v:
                return match.group(0)
            try:
                return f'{attr_name}="{float(v) * S}"'
            except ValueError:
                return match.group(0)
        return replacer

    content = re.sub(r'scale="([^"]+)"', _safe_scale_scale, content)
    content = re.sub(r'(<origin\s[^>]*)xyz="([^"]+)"', _safe_scale_xyz, content)
    content = re.sub(r'radius="([^"]+)"', _safe_scale_single('radius'), content)
    content = re.sub(r'length="([^"]+)"', _safe_scale_single('length'), content)
    content = re.sub(r'size="([^"]+)"', _safe_scale_size, content)

    # Change colors to white
    content = re.sub(r'(<color\s+rgba=")[^"]+"', r'\g<1>1.0 1.0 1.0 1.0"', content)

    with open(urdf_path, 'w') as f:
        f.write(content)
    print(f'  [URDF] Scaled {S}x uniform + white: {urdf_path}')


def main():
    if len(sys.argv) < 2:
        sys.exit(1)
    tb3_gz_dir = sys.argv[1]

    sdf_path = os.path.join(tb3_gz_dir, 'models', 'turtlebot3_waffle', 'model.sdf')
    if os.path.exists(sdf_path):
        scale_sdf(sdf_path)

    urdf_path = os.path.join(tb3_gz_dir, 'urdf', 'turtlebot3_waffle.urdf')
    if os.path.exists(urdf_path):
        scale_urdf(urdf_path)


if __name__ == '__main__':
    main()
