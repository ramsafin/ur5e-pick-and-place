#!/usr/bin/env python3
"""Standalone ikpy smoke test."""

from ikpy.chain import Chain

from ur5e_gazebo_demo.urdf_paths import get_ik_urdf_path


def main():
    chain = Chain.from_urdf_file(get_ik_urdf_path())
    target = [0.65, 0.25, 0.95]
    angles = chain.inverse_kinematics(target_position=target)
    print(angles)


if __name__ == '__main__':
    main()
