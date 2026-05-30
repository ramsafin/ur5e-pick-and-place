"""Resolve URDF paths for ikpy and offline tools."""

import os

from ament_index_python.packages import get_package_share_directory


def get_ik_urdf_path():
    """Return path to flat URDF used by ikpy."""
    share = get_package_share_directory('ur5e_gazebo_demo')
    candidates = [
        os.path.join(share, 'urdf', 'ur5e_full.urdf'),
        os.path.join(share, '..', '..', '..', 'ur5e_full.urdf'),
        os.path.join(share, '..', '..', '..', '..', 'ur5e_full.urdf'),
    ]
    for path in candidates:
        resolved = os.path.abspath(path)
        if os.path.isfile(resolved):
            return resolved
    raise FileNotFoundError(
        'ur5e_full.urdf not found; place it in ur5e_gazebo_demo/urdf/ or workspace root'
    )
