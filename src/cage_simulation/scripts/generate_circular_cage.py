#!/usr/bin/env python3
"""Generate the Gazebo circular cage from the same GPS geometry as RViz.

The generated model is visual/structural only: it is static, so it cannot
change the UAV dynamics.  World coordinates follow the ArduPilot Gazebo
convention (x=north, y=-east, z=up).
"""
import argparse
import math
import os
import re
import sys


HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, '..', '..', 'cage_geo', 'src')))
from cage_geo.geodesy import LocalENU  # noqa: E402


def scalar(path, key, cast=float):
    pattern = re.compile(r'^\s*' + re.escape(key) + r'\s*:\s*([^#\s]+)')
    with open(path, encoding='utf-8') as stream:
        for line in stream:
            match = pattern.match(line)
            if match:
                return cast(match.group(1).strip('"\''))
    raise KeyError('{} not found in {}'.format(key, path))


def build_model(center_x, center_y, radius, num_buoys, buoy_radius, buoy_height,
                endpoint_a=None, endpoint_b=None):
    lines = [
        '<?xml version="1.0"?>',
        '<sdf version="1.6">',
        '  <model name="circular_cage">',
        '    <static>true</static>',
        '    <pose>0 0 0 0 0 0</pose>',
        '    <link name="cage_link">',
    ]
    for index in range(num_buoys):
        theta = 2.0 * math.pi * index / num_buoys
        x = center_x + radius * math.cos(theta)
        y = center_y + radius * math.sin(theta)
        lines.extend([
            '      <collision name="buoy_{}_collision">'.format(index),
            '        <pose>{:.6f} {:.6f} {:.6f} 0 0 0</pose>'.format(
                x, y, buoy_height / 2.0),
            '        <geometry><cylinder><radius>{:.3f}</radius><length>{:.3f}</length>'
            '</cylinder></geometry>'.format(buoy_radius, buoy_height),
            '      </collision>',
            '      <visual name="buoy_{}_visual">'.format(index),
            '        <pose>{:.6f} {:.6f} {:.6f} 0 0 0</pose>'.format(
                x, y, buoy_height / 2.0),
            '        <geometry><cylinder><radius>{:.3f}</radius><length>{:.3f}</length>'
            '</cylinder></geometry>'.format(buoy_radius, buoy_height),
            '        <material><ambient>0.88 0.92 0.95 1</ambient>'
            '<diffuse>0.96 0.97 0.98 1</diffuse></material>',
            '      </visual>',
        ])
    for name, position, ambient, diffuse in (
            ('gps_a', endpoint_a, '0.10 0.72 0.24 1', '0.15 0.85 0.30 1'),
            ('gps_b', endpoint_b, '0.90 0.36 0.06 1', '1.0 0.50 0.10 1')):
        if position is None:
            continue
        x, y = position
        lines.extend([
            '      <collision name="{}_collision">'.format(name),
            '        <pose>{:.6f} {:.6f} 0.50 0 0 0</pose>'.format(x, y),
            '        <geometry><cylinder><radius>0.55</radius><length>1.0</length></cylinder></geometry>',
            '      </collision>',
            '      <visual name="{}_body">'.format(name),
            '        <pose>{:.6f} {:.6f} 0.50 0 0 0</pose>'.format(x, y),
            '        <geometry><cylinder><radius>0.55</radius><length>1.0</length></cylinder></geometry>',
            '        <material><ambient>{}</ambient><diffuse>{}</diffuse></material>'.format(ambient, diffuse),
            '      </visual>',
            '      <visual name="{}_top">'.format(name),
            '        <pose>{:.6f} {:.6f} 1.20 0 0 0</pose>'.format(x, y),
            '        <geometry><sphere><radius>0.38</radius></sphere></geometry>',
            '        <material><ambient>{}</ambient><diffuse>{}</diffuse></material>'.format(ambient, diffuse),
            '      </visual>',
        ])
    lines.extend(['    </link>', '  </model>', '</sdf>', ''])
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cage-config', default=None)
    parser.add_argument('--visualization-config', default=None)
    parser.add_argument('--gazebo-config', default=None)
    parser.add_argument('--output', default=None)
    parser.add_argument('--center-x', type=float)
    parser.add_argument('--center-y', type=float)
    parser.add_argument('--radius', type=float)
    parser.add_argument('--num-buoys', type=int)
    parser.add_argument('--buoy-radius', type=float)
    parser.add_argument('--buoy-height', type=float)
    args = parser.parse_args()

    workspace = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
    cage_config = args.cage_config or os.path.join(workspace, 'src/cage_bringup/config/cage.yaml')
    vis_config = args.visualization_config or os.path.join(workspace, 'src/cage_visualization/config/visualization.yaml')
    gazebo_config = args.gazebo_config or os.path.join(workspace, 'src/cage_simulation/config/gazebo.yaml')
    output = args.output or os.path.join(workspace, 'src/cage_simulation/models/circular_cage/model.sdf')

    num_buoys = args.num_buoys if args.num_buoys is not None else 48
    buoy_radius = args.buoy_radius if args.buoy_radius is not None else 0.35
    buoy_height = args.buoy_height if args.buoy_height is not None else 0.50
    if os.path.exists(gazebo_config):
        if args.num_buoys is None:
            num_buoys = scalar(gazebo_config, 'num_buoys', int)
        if args.buoy_radius is None:
            buoy_radius = scalar(gazebo_config, 'buoy_radius')
        if args.buoy_height is None:
            buoy_height = scalar(gazebo_config, 'buoy_height')

    endpoint_a = endpoint_b = None
    if args.center_x is None or args.center_y is None or args.radius is None:
        home = (scalar(vis_config, 'home_lat'), scalar(vis_config, 'home_lon'),
                scalar(vis_config, 'home_alt'))
        local = LocalENU(*home)
        a = local.to_enu(scalar(cage_config, 'endpoint_a_lat'),
                         scalar(cage_config, 'endpoint_a_lon'),
                         scalar(cage_config, 'endpoint_a_alt'))
        b = local.to_enu(scalar(cage_config, 'endpoint_b_lat'),
                         scalar(cage_config, 'endpoint_b_lon'),
                         scalar(cage_config, 'endpoint_b_alt'))
        endpoint_a = (a[1], -a[0])
        endpoint_b = (b[1], -b[0])
        center_east = (a[0] + b[0]) / 2.0
        center_north = (a[1] + b[1]) / 2.0
        center_x = center_north if args.center_x is None else args.center_x
        center_y = -center_east if args.center_y is None else args.center_y
        radius = math.hypot(b[0] - a[0], b[1] - a[1]) / 2.0 if args.radius is None else args.radius
    else:
        center_x, center_y, radius = args.center_x, args.center_y, args.radius

    if num_buoys < 8:
        raise SystemExit('num-buoys must be at least 8')
    if radius <= 0 or buoy_radius <= 0 or buoy_height <= 0:
        raise SystemExit('radius and buoy dimensions must be positive')
    os.makedirs(os.path.dirname(output), exist_ok=True)
    with open(output, 'w', encoding='utf-8') as stream:
        stream.write(build_model(center_x, center_y, radius, num_buoys,
                                 buoy_radius, buoy_height,
                                 endpoint_a, endpoint_b))
    print('generated {}: center=( {:.3f}, {:.3f} ), radius={:.3f}, buoys={}'.format(
        output, center_x, center_y, radius, num_buoys))


if __name__ == '__main__':
    main()
