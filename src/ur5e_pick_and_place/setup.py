from glob import glob
import os

from setuptools import setup

package_name = 'ur5e_pick_and_place'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shuai',
    maintainer_email='shuai@example.com',
    description='UR5e pick-and-place nodes',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pick_and_place = ur5e_pick_and_place.pick_and_place:main',
            'pick_and_place_moveit = ur5e_pick_and_place.pick_and_place_moveit:main',
            'vision_pick = ur5e_pick_and_place.vision_pick:main',
            'vision_pick_moveit = ur5e_pick_and_place.vision_pick_moveit:main',
            'validate_config = ur5e_pick_and_place.config_validation:main',
            'go_home = ur5e_pick_and_place.go_home:main',
        ],
    },
)
