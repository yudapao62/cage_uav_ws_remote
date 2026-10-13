#!/usr/bin/python3
"""Generate a deterministic, static fish-farm scene for Gazebo Classic.

The generator keeps the farm in one top-level model and uses simple boxes,
cylinders and low-poly OBJ strips so it remains suitable for real-time SITL.
Configuration coordinates are ENU metres; the ArduPilot Gazebo convention is
applied only when writing the SDF (world x=north, world y=-east).
"""
import argparse
from dataclasses import replace
import math
import os
import random
import sys

import yaml


HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, '..', '..', 'cage_grid', 'src')))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, '..', '..', 'cage_geo', 'src')))
from cage_grid.deformed_model import DeformationConfig, DeformedGrid  # noqa: E402


def world_from_enu(east, north, origin_east, origin_north, yaw_rad):
    rotated_east = origin_east + math.cos(yaw_rad) * east - math.sin(yaw_rad) * north
    rotated_north = origin_north + math.sin(yaw_rad) * east + math.cos(yaw_rad) * north
    return rotated_north, -rotated_east


def material(name, ambient, diffuse, specular='0.05 0.05 0.05 1'):
    return ('        <material><ambient>{}</ambient><diffuse>{}</diffuse>'
            '<specular>{}</specular></material>'.format(ambient, diffuse, specular))


def generate_net_obj(path, points, width, lateral_jitter, height_jitter, z, rng):
    """Write a low-poly horizontal wavy strip centered on a farm row."""
    vertices = []
    for index, (x, y) in enumerate(points):
        if index == 0:
            dx, dy = points[1][0] - x, points[1][1] - y
        elif index == len(points) - 1:
            dx, dy = x - points[index - 1][0], y - points[index - 1][1]
        else:
            dx = points[index + 1][0] - points[index - 1][0]
            dy = points[index + 1][1] - points[index - 1][1]
        length = max(math.hypot(dx, dy), 1e-9)
        nx, ny = -dy / length, dx / length
        jitter = rng.uniform(-lateral_jitter, lateral_jitter)
        z_jitter = rng.uniform(-height_jitter, height_jitter)
        left = width / 2.0 + jitter
        right = -width / 2.0 + jitter
        vertices.extend([
            (x + nx * left, y + ny * left, z + z_jitter),
            (x + nx * right, y + ny * right, z + z_jitter),
        ])
    with open(path, 'w', encoding='utf-8') as stream:
        stream.write('# generated deterministic fish-farm net strip\n')
        for vertex in vertices:
            stream.write('v {:.6f} {:.6f} {:.6f}\n'.format(*vertex))
        for index in range(0, len(vertices) - 2, 2):
            stream.write('f {} {} {} {}\n'.format(index + 1, index + 2,
                                                   index + 4, index + 3))


def generate(config_path, output_path, ground_truth_output=None, model_name='fish_farm',
             cage_id=0, geometry_seed=None, deformation_preset=None):
    with open(config_path, encoding='utf-8') as stream:
        config = yaml.safe_load(stream) or {}
    farm = config['fish_farm']
    rows = int(farm['rows'])
    cols = int(farm['cols'])
    cell_x = float(farm['cell_size']['x'])
    cell_y = float(farm['cell_size']['y'])
    origin_east = float(farm.get('origin_east', farm.get('origin_x', 0.0)))
    origin_north = float(farm.get('origin_north', farm.get('origin_y', 0.0)))
    yaw_rad = math.radians(float(farm.get('yaw_deg', 0.0)))
    lattice_geometry_mode = str(farm.get('lattice_geometry_mode', 'nominal')).strip().lower()
    if lattice_geometry_mode not in ('nominal', 'deformed'):
        raise ValueError('fish_farm.lattice_geometry_mode must be nominal or deformed')
    beam = farm.get('beam', {})
    beam_width = float(beam.get('width', 0.25))
    beam_height = float(beam.get('height', 0.20))
    beam_z = float(beam.get('z', 0.45))
    floats = farm.get('floats', {})
    floats_enabled = bool(floats.get('enabled', True))
    float_spacing = float(floats.get('spacing', 4.0))
    float_radius = float(floats.get('radius', 0.30))
    float_length = float(floats.get('length', 0.90))
    blue_ratio = float(floats.get('blue_ratio', 0.70))
    net_cfg = farm.get('net', {})
    net_enabled = bool(net_cfg.get('enabled', True))
    net_rows = [int(value) for value in net_cfg.get('rows', [1, 3, 5])]
    net_width = float(net_cfg.get('width', 1.20))
    net_z = float(net_cfg.get('z', 0.62))
    net_height_jitter = float(net_cfg.get('height_jitter', 0.08))
    net_lateral_jitter = float(net_cfg.get('lateral_jitter', 0.20))
    realism = farm.get('realism', {})
    rng = random.Random(int(realism.get('random_seed', 42)))
    missing_probability = float(realism.get('float_missing_probability', 0.03))

    mesh_dir = os.path.join(os.path.dirname(output_path), 'meshes')
    os.makedirs(mesh_dir, exist_ok=True)
    deformation_mapping = dict(farm.get('deformation', {}))
    if deformation_preset is not None:
        deformation_mapping['preset'] = str(deformation_preset)
    deformation = DeformationConfig.from_mapping(deformation_mapping)
    if geometry_seed is not None:
        deformation = replace(deformation, random_seed=int(geometry_seed))
    if lattice_geometry_mode == 'nominal':
        deformation = replace(deformation, enabled=False)
    deformed_grid = DeformedGrid(
        rows=rows,
        cols=cols,
        cell_size_x=cell_x,
        cell_size_y=cell_y,
        origin_east=origin_east,
        origin_north=origin_north,
        origin_up=0.0,
        yaw_deg=math.degrees(yaw_rad),
        cage_id=int(cage_id),
        config=deformation,
        structural_holes=farm.get('structural_holes', []),
    )
    grid = {
        index: world_from_enu(vertex.east, vertex.north, 0.0, 0.0, 0.0)
        for index, vertex in deformed_grid.vertices.items()
    }

    lines = [
        '<?xml version="1.0"?>',
        '<sdf version="1.6">',
        '  <model name="{}">'.format(model_name),
        '    <static>true</static>',
        '    <pose>0 0 0 0 0 0</pose>',
        '    <link name="farm_link">',
    ]
    beam_index = 0
    row_paths = {}

    def add_segment(p1, p2, label):
        nonlocal beam_index
        x1, y1 = p1
        x2, y2 = p2
        dx, dy = x2 - x1, y2 - y1
        length = max(math.hypot(dx, dy), 1e-6)
        # Structural beams must follow the exact shared-vertex GT.  Appearance
        # randomness belongs to floats/materials/nets, not task geometry.
        angle = math.atan2(dy, dx)
        mid_x, mid_y = (x1 + x2) / 2.0, (y1 + y2) / 2.0
        lines.extend([
            '      <collision name="beam_{}_collision">'.format(beam_index),
            '        <pose>{:.6f} {:.6f} {:.6f} 0 0 {:.6f}</pose>'.format(
                mid_x, mid_y, beam_z, angle),
            '        <geometry><box><size>{:.6f} {:.6f} {:.6f}</size></box></geometry>'.format(
                length, beam_width, beam_height),
            '      </collision>',
            '      <visual name="beam_{}_visual">'.format(beam_index),
            '        <pose>{:.6f} {:.6f} {:.6f} 0 0 {:.6f}</pose>'.format(
                mid_x, mid_y, beam_z, angle),
            '        <geometry><box><size>{:.6f} {:.6f} {:.6f}</size></box></geometry>'.format(
                length, beam_width, beam_height),
            material('beam', '0.62 0.64 0.63 1', '0.76 0.78 0.77 1', '0.08 0.08 0.08 1'),
            '      </visual>',
        ])
        beam_index += 1
        return length, angle, mid_x, mid_y

    # Horizontal rows and their intermittent floats.
    for row in range(rows + 1):
        row_paths[row] = []
        for col in range(cols):
            p1, p2 = grid[(row, col)], grid[(row, col + 1)]
            row_paths[row].append(p1)
            if col == cols - 1:
                row_paths[row].append(p2)
            length, angle, _, _ = add_segment(p1, p2, 'row_{}_{}'.format(row, col))
            if floats_enabled:
                count = max(1, int(length / float_spacing))
                for float_index in range(1, count):
                    if rng.random() < missing_probability:
                        continue
                    ratio = float_index / float(count)
                    x = p1[0] + ratio * (p2[0] - p1[0])
                    y = p1[1] + ratio * (p2[1] - p1[1])
                    color_name = 'blue' if rng.random() < blue_ratio else 'white'
                    if color_name == 'blue':
                        ambient, diffuse = '0.08 0.24 0.42 1', '0.12 0.34 0.58 1'
                    else:
                        ambient, diffuse = '0.70 0.73 0.72 1', '0.84 0.85 0.82 1'
                    lines.extend([
                        '      <visual name="float_row_{}_{}_{}">'.format(row, col, float_index),
                        '        <pose>{:.6f} {:.6f} {:.6f} 0 {:.6f} {:.6f}</pose>'.format(
                            x, y, beam_z + 0.16, math.pi / 2.0, angle),
                        '        <geometry><cylinder><radius>{:.3f}</radius><length>{:.3f}</length></cylinder></geometry>'.format(
                            float_radius, float_length),
                        material('float', ambient, diffuse),
                        '      </visual>',
                    ])

    # Vertical grid lines.
    for col in range(cols + 1):
        for row in range(rows):
            add_segment(grid[(row, col)], grid[(row + 1, col)],
                        'col_{}_{}'.format(col, row))

    # Generate selected net rows as low-poly strip meshes.
    if net_enabled:
        for row in net_rows:
            if row < 0 or row > rows:
                continue
            path_points = []
            for col in range(cols + 1):
                if col == 0:
                    path_points.append(grid[(row, col)])
                else:
                    prev = grid[(row, col - 1)]
                    current = grid[(row, col)]
                    distance = math.hypot(current[0] - prev[0], current[1] - prev[1])
                    steps = max(1, int(distance / 0.5))
                    for step in range(1, steps + 1):
                        ratio = step / float(steps)
                        path_points.append((prev[0] + ratio * (current[0] - prev[0]),
                                            prev[1] + ratio * (current[1] - prev[1])))
            mesh_name = 'net_row_{}.obj'.format(row)
            generate_net_obj(os.path.join(mesh_dir, mesh_name), path_points,
                             net_width, net_lateral_jitter, net_height_jitter,
                             net_z, rng)
            lines.extend([
                '      <visual name="net_row_{}">'.format(row),
                '        <geometry><mesh><uri>model://{}/meshes/{}</uri></mesh></geometry>'.format(model_name, mesh_name),
                '        <transparency>0.18</transparency>',
                '        <cast_shadows>false</cast_shadows>',
                material('net', '0.05 0.26 0.14 1', '0.08 0.42 0.20 1', '0.01 0.03 0.01 1'),
                '      </visual>',
            ])
    lines.extend(['    </link>', '  </model>', '</sdf>', ''])
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, 'w', encoding='utf-8') as stream:
        stream.write('\n'.join(lines))
    if ground_truth_output is None:
        package_dir = os.path.abspath(os.path.join(os.path.dirname(output_path), '..', '..'))
        ground_truth_output = os.path.join(package_dir, 'generated', '{}_gt.json'.format(model_name))
    os.makedirs(os.path.dirname(ground_truth_output), exist_ok=True)
    manifest = deformed_grid.write_manifest(ground_truth_output)
    return {'rows': rows, 'cols': cols, 'beams': beam_index,
            'nets': len([row for row in net_rows if 0 <= row <= rows]),
            'output': output_path, 'ground_truth': ground_truth_output,
            'geometry_sha256': manifest['geometry_sha256']}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--ground-truth-output')
    parser.add_argument('--model-name', default='fish_farm')
    parser.add_argument('--cage-id', type=int, default=0)
    parser.add_argument('--geometry-seed', type=int)
    parser.add_argument('--deformation-preset', choices=('mild', 'medium', 'strong'))
    args = parser.parse_args()
    result = generate(args.config, args.output, args.ground_truth_output,
                      args.model_name, args.cage_id, args.geometry_seed,
                      args.deformation_preset)
    print('generated fish_farm rows={rows} cols={cols} beams={beams} nets={nets}: '
          '{output}; gt={ground_truth}; sha256={geometry_sha256}'.format(**result))


if __name__ == '__main__':
    main()
