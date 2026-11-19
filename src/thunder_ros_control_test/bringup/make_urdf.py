#!/usr/bin/env python3
"""URDF for the tests: the chain base..tip of a URDF, dynamics only.

- keeps the links from base to tip and the joints between them, without visual and
  collision (no meshes needed), so the simulated robot is the one Thunder models;
- fixes base to the world;
- adds a ros2_control block: effort command; position, velocity, effort state;
- for Gazebo (gz_ros2_control), the plugin with the controller manager parameters.
"""
import argparse
import xml.etree.ElementTree as ET


def chain(root, base, tip):
    joints = {j.find('child').get('link'): j for j in root.findall('joint')}
    links, kept_joints = [tip], []
    while links[-1] != base:
        joint = joints[links[-1]]
        kept_joints.append(joint)
        links.append(joint.find('parent').get('link'))
    return list(reversed(links)), list(reversed(kept_joints))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('urdf')
    parser.add_argument('output')
    parser.add_argument('--base', required=True)
    parser.add_argument('--tip', required=True)
    parser.add_argument('--hardware', choices=['gz', 'mock'], default='gz')
    parser.add_argument('--initial', type=float, nargs='*', default=[], help='initial position of each moving joint')
    parser.add_argument('--controllers', default='', help='controller manager parameters (gz only)')
    args = parser.parse_args()

    source = ET.parse(args.urdf).getroot()
    links, joints = chain(source, args.base, args.tip)

    robot = ET.Element('robot', name=source.get('name'))
    ET.SubElement(robot, 'link', name='world')
    world = ET.SubElement(robot, 'joint', name='world_joint', type='fixed')
    ET.SubElement(world, 'parent', link='world')
    ET.SubElement(world, 'child', link=args.base)
    for link in source.findall('link'):
        if link.get('name') in links:
            for tag in ('visual', 'collision'):
                for element in link.findall(tag):
                    link.remove(element)
            robot.append(link)
    moving = []
    for joint in joints:
        robot.append(joint)
        if joint.get('type') != 'fixed':
            moving.append(joint.get('name'))

    if args.initial and len(args.initial) != len(moving):
        parser.error(f'--initial needs {len(moving)} values')
    control = ET.SubElement(robot, 'ros2_control', name='TestSystem', type='system')
    hardware = ET.SubElement(control, 'hardware')
    plugin = 'gz_ros2_control/GazeboSimSystem' if args.hardware == 'gz' else 'mock_components/GenericSystem'
    ET.SubElement(hardware, 'plugin').text = plugin
    for i, name in enumerate(moving):
        joint = ET.SubElement(control, 'joint', name=name)
        ET.SubElement(joint, 'command_interface', name='effort')
        position = ET.SubElement(joint, 'state_interface', name='position')
        ET.SubElement(position, 'param', name='initial_value').text = str(args.initial[i] if args.initial else 0.0)
        ET.SubElement(joint, 'state_interface', name='velocity')
        ET.SubElement(joint, 'state_interface', name='effort')

    if args.hardware == 'gz':
        gazebo = ET.SubElement(robot, 'gazebo')
        plugin = ET.SubElement(gazebo, 'plugin', filename='gz_ros2_control-system',
                               name='gz_ros2_control::GazeboSimROS2ControlPlugin')
        ET.SubElement(plugin, 'parameters').text = args.controllers

    ET.indent(robot)
    ET.ElementTree(robot).write(args.output, encoding='unicode', xml_declaration=True)
    print(f'{args.output}: links {links}, moving joints {moving}')


if __name__ == '__main__':
    main()
