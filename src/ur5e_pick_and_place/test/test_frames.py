from ur5e_pick_and_place.frames import base_to_world, pixel_to_base, world_to_base


def test_world_to_base_red_box():
    bx, by, bz = world_to_base(0.75, 0.0, 0.925)
    assert abs(bx - 0.0) < 1e-6
    assert abs(by - 0.75) < 1e-6
    assert abs(bz - 0.125) < 1e-6


def test_base_to_world_roundtrip():
    wx, wy, wz = base_to_world(0.0, 0.75, 0.125)
    assert abs(wx - 0.75) < 1e-6
    assert abs(wy - 0.0) < 1e-6
    assert abs(wz - 0.925) < 1e-6


def test_pixel_to_base_at_image_center():
    bx, by = pixel_to_base(
        320, 240,
        center_x=320, center_y=240,
        world_x_at_center=0.0, world_y_at_center=0.0,
        scale_x=0.00283, scale_y=0.00283,
        table_world_z=0.925,
    )
    assert abs(bx - 0.0) < 1e-6
    assert abs(by - 0.0) < 1e-6


def test_pixel_to_base_at_red_box():
    """Measured centroid for box_object at world (0.75, 0) in camera_world.sdf."""
    bx, by = pixel_to_base(
        585, 240,
        center_x=320, center_y=240,
        world_x_at_center=0.0, world_y_at_center=0.0,
        scale_x=0.00283, scale_y=0.00283,
        table_world_z=0.925,
    )
    assert abs(bx - 0.0) < 0.02
    assert abs(by - 0.75) < 0.02
