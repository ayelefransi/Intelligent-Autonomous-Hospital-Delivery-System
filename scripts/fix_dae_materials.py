#!/usr/bin/env python3
"""
fix_dae_materials.py — Add PBR material tags to DAE-based Gazebo models.
Prevents gray/white rendering in Gazebo Harmonic's OGRE2 renderer.
"""
import os, re, sys

BASE = sys.argv[1] if len(sys.argv) > 1 else "."

def find_texture_in_dae(dae_path):
    """Extract texture filename from DAE's <init_from> tag."""
    try:
        with open(dae_path, 'r', encoding='utf-8', errors='ignore') as f:
            match = re.search(r'<init_from>([^<]+)</init_from>', f.read())
        return match.group(1) if match else None
    except Exception:
        return None

def fix_model_sdf(sdf_path, model_name, albedo_tex):
    """Add PBR material block to an SDF model file."""
    with open(sdf_path, 'r', encoding='utf-8', errors='ignore') as f:
        content = f.read()

    if '<material>' in content or '<pbr>' in content:
        return False  # Already has material

    if not albedo_tex:
        return False

    material_block = (
        '      <material>\n'
        '        <pbr>\n'
        '          <metal>\n'
        f'            <emissive_map>model://{model_name}/meshes/{albedo_tex}</emissive_map>\n'
        '            <roughness>0.5</roughness>\n'
        '            <metalness>0.0</metalness>\n'
        '          </metal>\n'
        '        </pbr>\n'
        '      </material>\n'
    )

    new_content = content.replace(
        '<visual name="visual">',
        '<visual name="visual">\n' + material_block,
        1
    )

    if new_content != content:
        with open(sdf_path, 'w', encoding='utf-8') as f:
            f.write(new_content)
        return True
    return False

fixed_count = 0
for models_container in ['models', 'fuel_models']:
    dirpath = os.path.join(BASE, models_container)
    if not os.path.isdir(dirpath):
        continue

    for model_name in os.listdir(dirpath):
        sdf_path = os.path.join(dirpath, model_name, 'model.sdf')
        mesh_dir = os.path.join(dirpath, model_name, 'meshes')

        if not os.path.isfile(sdf_path) or not os.path.isdir(mesh_dir):
            continue

        dae_files = [f for f in os.listdir(mesh_dir) if f.endswith('.dae')]
        if not dae_files:
            continue

        albedo_tex = None
        # Prefer visual DAE files
        for dae in dae_files:
            if 'visual' in dae.lower():
                albedo_tex = find_texture_in_dae(os.path.join(mesh_dir, dae))
                if albedo_tex:
                    break

        if not albedo_tex:
            albedo_tex = find_texture_in_dae(os.path.join(mesh_dir, dae_files[0]))

        if fix_model_sdf(sdf_path, model_name, albedo_tex):
            print(f"  FIXED: {model_name} -> {albedo_tex}")
            fixed_count += 1

print(f"Fixed {fixed_count} DAE models with PBR materials.")
