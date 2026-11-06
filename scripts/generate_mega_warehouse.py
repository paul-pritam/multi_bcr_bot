#!/usr/bin/env python3
"""
Procedural Mega-Warehouse SDF Generator for BCR Bot / Gazebo Harmonic.

Generates an industrial scale 120m x 80m logistics distribution center
using existing AWS RoboMaker warehouse assets.
"""

import math
import os


def generate_mega_warehouse_sdf(
    output_path: str,
    length_x: float = 120.0,
    width_y: float = 80.0
):
    """
    Generate an industrial SDF world model with perimeter walls, aisles, and clutter.

    :param output_path: Destination path for .sdf file.
    :param length_x: Facility length in meters.
    :param width_y: Facility width in meters.
    """
    sdf = []
    sdf.append('<?xml version="1.0" ?>')
    sdf.append('<sdf version="1.6">')
    sdf.append('  <world name="mega_warehouse">')
    sdf.append('    <physics name="1ms" type="ignored">')
    sdf.append('      <max_step_size>0.003</max_step_size>')
    sdf.append('      <real_time_factor>1.0</real_time_factor>')
    sdf.append('    </physics>')
    sdf.append(
        '    <plugin filename="gz-sim-physics-system" '
        'name="gz::sim::systems::Physics"></plugin>')
    sdf.append(
        '    <plugin filename="gz-sim-user-commands-system" '
        'name="gz::sim::systems::UserCommands"></plugin>')
    sdf.append(
        '    <plugin filename="gz-sim-scene-broadcaster-system" '
        'name="gz::sim::systems::SceneBroadcaster"></plugin>')
    sdf.append(
        '    <plugin filename="gz-sim-sensors-system" '
        'name="gz::sim::systems::Sensors">')
    sdf.append('      <render_engine>ogre2</render_engine>')
    sdf.append('    </plugin>')
    sdf.append(
        '    <plugin filename="gz-sim-imu-system" '
        'name="gz::sim::systems::Imu"></plugin>')
    sdf.append('    <scene>')
    sdf.append('      <ambient>0.6 0.6 0.6 1.0</ambient>')
    sdf.append('      <background>0.15 0.15 0.15 1.0</background>')
    sdf.append('      <grid>false</grid>')
    sdf.append('    </scene>')
    sdf.append('    <light type="directional" name="sun">')
    sdf.append('      <cast_shadows>true</cast_shadows>')
    sdf.append('      <pose>0 0 30 0 0 0</pose>')
    sdf.append('      <diffuse>0.8 0.8 0.8 1</diffuse>')
    sdf.append('      <specular>0.2 0.2 0.2 1</specular>')
    sdf.append('      <direction>-0.5 0.1 -0.9</direction>')
    sdf.append('    </light>')

    # Ground Plane
    sdf.append('    <model name="ground_plane">')
    sdf.append('      <static>true</static>')
    sdf.append('      <link name="link">')
    sdf.append('        <collision name="collision">')
    sdf.append(
        '          <geometry><plane><normal>0 0 1</normal>'
        '<size>250 250</size></plane></geometry>')
    sdf.append('        </collision>')
    sdf.append('        <visual name="visual">')
    sdf.append(
        '          <geometry><plane><normal>0 0 1</normal>'
        '<size>250 250</size></plane></geometry>')
    sdf.append('          <material>')
    sdf.append('            <ambient>0.3 0.3 0.35 1</ambient>')
    sdf.append('            <diffuse>0.4 0.4 0.45 1</diffuse>')
    sdf.append('          </material>')
    sdf.append('        </visual>')
    sdf.append('      </link>')
    sdf.append('    </model>')

    model_idx = 1

    # Perimeter Walls (120m x 80m box with 4m height)
    half_x = length_x / 2.0
    half_y = width_y / 2.0
    wall_thickness = 0.3
    wall_height = 4.0

    walls = [
        ('wall_north', 0.0, half_y, length_x, wall_thickness),
        ('wall_south', 0.0, -half_y, length_x, wall_thickness),
        ('wall_east', half_x, 0.0, wall_thickness, width_y),
        ('wall_west', -half_x, 0.0, wall_thickness, width_y),
    ]

    for name, wall_x, wall_y, size_x, size_y in walls:
        sdf.append(f'    <model name="{name}">')
        sdf.append('      <static>true</static>')
        sdf.append(f'      <pose>{wall_x:.2f} {wall_y:.2f} {wall_height/2.0:.2f} 0 0 0</pose>')
        sdf.append('      <link name="link">')
        sdf.append('        <collision name="col">')
        sdf.append(
            f'          <geometry><box><size>{size_x:.2f} {size_y:.2f} '
            f'{wall_height:.2f}</size></box></geometry>')
        sdf.append('        </collision>')
        sdf.append('        <visual name="vis">')
        sdf.append(
            f'          <geometry><box><size>{size_x:.2f} {size_y:.2f} '
            f'{wall_height:.2f}</size></box></geometry>')
        sdf.append(
            '          <material><ambient>0.7 0.7 0.75 1</ambient>'
            '<diffuse>0.7 0.7 0.75 1</diffuse></material>')
        sdf.append('        </visual>')
        sdf.append('      </link>')
        sdf.append('    </model>')

    # Internal Shelving Layout:
    # 8 Longitudinal Aisles (along X from -45m to +45m)
    # Rack rows spaced by 5.5m in Y
    # Aisles have breaks every 25m along X for cross-traffic
    aisle_segment_starts_x = [-45.0, -18.0, 10.0]

    shelf_types = [
        'aws_robomaker_warehouse_ShelfD_01',
        'aws_robomaker_warehouse_ShelfE_01',
        'aws_robomaker_warehouse_ShelfF_01'
    ]

    aisle_positions_y = [-30.0, -22.0, -14.0, -6.0, 6.0, 14.0, 22.0, 30.0]

    for aisle_y in aisle_positions_y:
        # Create double-backed rack pairs with 1.2m gap between backs
        for delta_y in [-0.6, 0.6]:
            rack_y = aisle_y + delta_y
            for seg_start_x in aisle_segment_starts_x:
                shelves_per_segment = 8
                spacing_x = 2.1
                for shelf_sub_idx in range(shelves_per_segment):
                    rack_x = seg_start_x + shelf_sub_idx * spacing_x
                    shelf_model = shelf_types[model_idx % len(shelf_types)]
                    yaw = 0.0 if delta_y > 0 else math.pi
                    sdf.append(f'    <model name="shelf_{model_idx:04d}">')
                    sdf.append(
                        f'      <include><uri>model://{shelf_model}</uri></include>')
                    sdf.append(
                        f'      <pose>{rack_x:.2f} {rack_y:.2f} 0 0 0 {yaw:.4f}</pose>')
                    sdf.append('    </model>')
                    model_idx += 1

    # Inbound / Outbound Pallet Staging Areas at East/West ends
    clutter_types = [
        'aws_robomaker_warehouse_ClutteringA_01',
        'aws_robomaker_warehouse_ClutteringC_01',
        'aws_robomaker_warehouse_ClutteringD_01',
        'aws_robomaker_warehouse_PalletJackB_01'
    ]

    # Staging stations along West wall (Inbound Dock)
    for i, staging_y in enumerate(range(-25, 26, 10)):
        clutter_model = clutter_types[i % len(clutter_types)]
        sdf.append(f'    <model name="staging_west_{i+1}">')
        sdf.append(
            f'      <include><uri>model://{clutter_model}</uri></include>')
        sdf.append(f'      <pose>-52.0 {staging_y:.2f} 0 0 0 1.5708</pose>')
        sdf.append('    </model>')

    # Sorting & shipping staging along East wall
    for i, staging_y in enumerate(range(-25, 26, 10)):
        clutter_model = clutter_types[(i + 2) % len(clutter_types)]
        sdf.append(f'    <model name="staging_east_{i+1}">')
        sdf.append(
            f'      <include><uri>model://{clutter_model}</uri></include>')
        sdf.append(f'      <pose>52.0 {staging_y:.2f} 0 0 0 -1.5708</pose>')
        sdf.append('    </model>')

    sdf.append('  </world>')
    sdf.append('</sdf>')

    content = '\n'.join(sdf)
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, 'w', encoding='utf-8') as f:
        f.write(content)
    total_models = model_idx + 12
    area = length_x * width_y
    print(f'Generated Mega-Warehouse SDF at: {output_path}')
    print(f'Dimensions: {length_x}m x {width_y}m (Area: {area:.0f} m^2)')
    print(f'Total placed shelf/clutter models: {total_models}')


if __name__ == '__main__':
    target = '/home/pritam/bcr_ws/src/bcr_bot/worlds/mega_warehouse.sdf'
    generate_mega_warehouse_sdf(target, 120.0, 80.0)
