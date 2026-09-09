from setuptools import setup
import os
from glob import glob

package_name = 'mpc_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shirui',
    maintainer_email='shirui@idt.local',
    description='9-UAV distributed MPC controller for PX4 SITL (acados-based)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arming_node          = mpc_control.arming_node:main',
            'virtual_leader_node  = mpc_control.virtual_leader_node:main',
            'mpc_node             = mpc_control.mpc_node:main',
            'leader_node          = mpc_control.leader_node:main',
            'mocap_bridge_node    = mpc_control.mocap_bridge_node:main',
            'sitl_mocap_source_node = mpc_control.sitl_mocap_source_node:main',
            'static_pose_source_node = mpc_control.static_pose_source_node:main',
        ],
    },
)
