import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'pickasso_amr_2d'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jerry',
    maintainer_email='jerry@todo.todo',
    description='Pickasso AMR 2D Simulation',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'map_node                = pickasso_amr_2d.map_node:main',
            'planner_node           = pickasso_amr_2d.planner_node:main',
            'robot_sim_node         = pickasso_amr_2d.robot_sim_node:main',
            'robot_marker_node      = pickasso_amr_2d.robot_marker_node:main',
            'trajectory_generator_node = pickasso_amr_2d.trajectory_generator_node:main',
            'trajectory_follower_node  = pickasso_amr_2d.trajectory_follower_node:main',
            'station_marker_node    = pickasso_amr_2d.station_marker_node:main',
            'test_goal_node         = pickasso_amr_2d.test_goal_node:main',
            'path_to_segments_node  = pickasso_amr_2d.path_to_segments_node:main',
            'path_follower_node     = pickasso_amr_2d.path_follower_node:main',
            'cmd_vel_udp_bridge_node= pickasso_amr_2d.cmd_vel_udp_bridge_node:main',
            # ── Nuevo nodo de visión ArUco ──
            'aruco_obstacle_node    = pickasso_amr_2d.aruco_obstacle_node:main',
        ],
    },
)
