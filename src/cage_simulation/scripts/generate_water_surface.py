#!/usr/bin/python3
"""Generate the static low-saturation water model from water.yaml."""
import argparse
import os
import yaml


def generate(config_path, output_path):
    with open(config_path, encoding='utf-8') as stream:
        water = (yaml.safe_load(stream) or {}).get('water', {})
    sx = float(water.get('size_x', 2000.0))
    sy = float(water.get('size_y', 2000.0))
    color = water.get('color', {})
    ambient = water.get('ambient', {})
    reflection = float(water.get('reflection_strength', 0.15))
    sdf = '''<?xml version="1.0"?>
<sdf version="1.6">
  <model name="water_surface">
    <static>true</static>
    <link name="water_link">
      <collision name="water_collision">
        <geometry><plane><normal>0 0 1</normal><size>{sx:.3f} {sy:.3f}</size></plane></geometry>
      </collision>
      <visual name="water_visual">
        <geometry><plane><normal>0 0 1</normal><size>{sx:.3f} {sy:.3f}</size></plane></geometry>
        <material>
          <ambient>{ar:.3f} {ag:.3f} {ab:.3f} 1</ambient>
          <diffuse>{r:.3f} {g:.3f} {b:.3f} {a:.3f}</diffuse>
          <specular>{reflection:.3f} {reflection:.3f} {reflection:.3f} 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>
'''.format(sx=sx, sy=sy, ar=float(ambient.get('r', 0.24)),
           ag=float(ambient.get('g', 0.34)), ab=float(ambient.get('b', 0.35)),
           r=float(color.get('r', 0.35)), g=float(color.get('g', 0.47)),
           b=float(color.get('b', 0.49)), a=float(color.get('a', 1.0)),
           reflection=reflection)
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, 'w', encoding='utf-8') as stream:
        stream.write(sdf)
    print('generated water_surface {}x{}: {}'.format(sx, sy, output_path))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    generate(args.config, args.output)
