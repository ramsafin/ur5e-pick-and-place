from ikpy.chain import Chain
import numpy as np

chain = Chain.from_urdf_file(
    "/home/shuai/ur5e_ws/ur5e_full.urdf"
)

target = [
    0.65,
    0.25,
    0.95
]

angles = chain.inverse_kinematics(
    target_position=target
)

print(angles)
