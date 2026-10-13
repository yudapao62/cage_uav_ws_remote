#!/usr/bin/python3
"""Generate the gimbal model with a configurable Gazebo ROS camera sensor."""
import argparse
import math
import os
import yaml


def generate(config_path, output_path):
    with open(config_path, encoding='utf-8') as stream:
        config = yaml.safe_load(stream) or {}
    camera = config.get('camera', {})
    mode = camera.get('active_mode', 'debug')
    profile = camera.get(mode, camera.get('debug', {}))
    width = int(profile.get('width', 1280))
    height = int(profile.get('height', 720))
    fps = float(profile.get('fps', 30))
    fov = math.radians(float(profile.get('horizontal_fov_deg', 78.0)))
    mount = camera.get('mount', {})
    roll = math.radians(float(mount.get('roll_deg', 0.0)))
    # The configuration uses the aviation convention requested by the scene
    # design document: negative pitch means "look down".  Gazebo camera
    # sensors use +X as their optical axis, so the SDF Y-axis rotation has the
    # opposite sign for the same physical direction.
    pitch = -math.radians(float(mount.get('pitch_deg', -65.0)))
    yaw = math.radians(float(mount.get('yaw_deg', 0.0)))
    x, y, z = (float(mount.get(key, default)) for key, default in
               (('x', 0.0), ('y', 0.0), ('z', -0.08)))
    namespace = str(camera.get('topic_namespace', '/uav_camera'))
    image_topic = str(camera.get('image_topic', 'image_raw'))
    info_topic = str(camera.get('camera_info_topic', 'camera_info'))
    sdf = '''<?xml version="1.0"?>
<sdf version="1.6">
  <model name="gimbal_small_2d">
    <link name="base_link">
      <inertial><mass>0.01</mass><inertia><ixx>0.00001</ixx><iyy>0.00001</iyy><izz>0.00001</izz></inertia></inertial>
      <visual name="visual">
        <geometry><box><size>0.08 0.08 0.05</size></box></geometry>
        <material><ambient>0.08 0.08 0.08 1</ambient><diffuse>0.12 0.12 0.12 1</diffuse></material>
      </visual>
      <sensor name="uav_camera" type="camera">
        <pose>{x:.6f} {y:.6f} {z:.6f} {roll:.6f} {pitch:.6f} {yaw:.6f}</pose>
        <camera>
          <horizontal_fov>{fov:.9f}</horizontal_fov>
          <image><width>{width}</width><height>{height}</height><format>R8G8B8</format></image>
          <clip><near>0.10</near><far>1000.0</far></clip>
        </camera>
        <always_on>true</always_on>
        <update_rate>{fps:.3f}</update_rate>
        <visualize>true</visualize>
        <plugin name="uav_camera_controller" filename="libgazebo_ros_camera.so">
          <robotNamespace>{namespace}</robotNamespace>
          <cameraName>front</cameraName>
          <imageTopicName>{image_topic}</imageTopicName>
          <cameraInfoTopicName>{info_topic}</cameraInfoTopicName>
          <frameName>uav_camera_optical_frame</frameName>
        </plugin>
      </sensor>
    </link>
  </model>
</sdf>
'''.format(x=x, y=y, z=z, roll=roll, pitch=pitch, yaw=yaw,
           fov=fov, width=width, height=height, fps=fps,
           namespace=namespace, image_topic=image_topic,
           info_topic=info_topic)
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, 'w', encoding='utf-8') as stream:
        stream.write(sdf)
    print('generated UAV camera mode={} {}x{} {:.1f}fps fov={:.1f}deg pitch={:.1f}deg: {}'.format(
        mode, width, height, fps, math.degrees(fov), math.degrees(pitch), output_path))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    generate(args.config, args.output)
