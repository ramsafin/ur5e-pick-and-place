"""Coordinate transforms for the UR5e table-mounted simulation."""

ROBOT_MOUNT_Z = 0.80


def world_to_base(world_x, world_y, world_z, mount_z=ROBOT_MOUNT_Z):
    """Convert Gazebo world coordinates to base_link."""
    return (-world_y, world_x, world_z - mount_z)


def base_to_world(base_x, base_y, base_z, mount_z=ROBOT_MOUNT_Z):
    """Convert base_link coordinates to Gazebo world."""
    return (base_y, -base_x, base_z + mount_z)


def pixel_to_world(cx, cy, center_x, center_y, world_x_at_center,
                   world_y_at_center, scale_x, scale_y):
    """Map image pixel centroid to world XY on the table plane."""
    world_x = world_x_at_center + (cx - center_x) * scale_x
    world_y = world_y_at_center + (cy - center_y) * scale_y
    return world_x, world_y


def pixel_to_base(cx, cy, center_x, center_y, world_x_at_center,
                  world_y_at_center, scale_x, scale_y,
                  table_world_z=0.925, mount_z=ROBOT_MOUNT_Z):
    """Map image pixel centroid to base_link XY on the table plane."""
    wx, wy = pixel_to_world(
        cx, cy, center_x, center_y,
        world_x_at_center, world_y_at_center, scale_x, scale_y,
    )
    bx, by, _ = world_to_base(wx, wy, table_world_z, mount_z)
    return bx, by
