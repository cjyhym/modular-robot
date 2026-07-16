import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'erobot_motion_client'

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
    maintainer='hust',
    maintainer_email='hust@example.com',
    description='Motion client for erobot EtherCAT arm',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ptp_joint_client = erobot_motion_client.ptp_joint_client:main',
            'ee_pose_goal = erobot_motion_client.ee_pose_goal:main',
            'linear_interpolation_client = erobot_motion_client.linear_interpolation_client:main',
            'arc_interpolation_client = erobot_motion_client.arc_interpolation_client:main',
            'torque_monitor = erobot_motion_client.torque_monitor:main',
        ],
    },
)
