from setuptools import find_packages, setup

package_name = 'turtle_nav'

import os
from glob import glob

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='joel',
    maintainer_email='joelgeorgekal@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'plan_to_control = turtle_nav.plan_to_control:main',
            'waypoint_navigator = turtle_nav.waypoint_navigator:main',
            'waypoint_navigator_no_bt_r1 = turtle_nav.waypoint_navigator_no_bt_r1:main',
            'waypoint_navigator_no_bt_r2 = turtle_nav.waypoint_navigator_no_bt_r2:main',
            'waypoint_navigator_no_bt_r3 = turtle_nav.waypoint_navigator_no_bt_r3:main',
            'waypoint_navigator_no_bt_r4 = turtle_nav.waypoint_navigator_no_bt_r4:main',
            'waypoint_navigator_no_bt_r1_smooth = turtle_nav.waypoint_navigator_no_bt_r1_smooth:main',
            'waypoint_navigator_no_bt_r4_smooth = turtle_nav.waypoint_navigator_no_bt_r4_smooth:main',
            'waypoint_navigator_no_bt_path_through_poses_r1 = turtle_nav.waypoint_navigator_no_bt_path_through_poses_r1:main',
            'waypoint_navigator_no_bt_path_through_poses_r2 = turtle_nav.waypoint_navigator_no_bt_path_through_poses_r2:main',
            'homicidal_chauffeur_game = turtle_nav.homicidal_chauffeur_game:main',
            'evader = turtle_nav.evader:main',
            'chauffeur = turtle_nav.chauffeur:main'
        ],
    },
)
