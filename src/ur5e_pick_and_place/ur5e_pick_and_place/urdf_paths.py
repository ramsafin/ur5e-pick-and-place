"""Resolve URDF paths for ikpy and offline tools."""

import os

from ament_index_python.packages import get_package_share_directory


def get_ik_urdf_path():
    """Return path to flat URDF used by ikpy."""
    share = get_package_share_directory('ur5e_pick_and_place')
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
        'ur5e_full.urdf not found; run: '
        'bash $(ros2 pkg prefix ur5e_description)/share/ur5e_description/scripts/generate_urdf.sh'
    )
