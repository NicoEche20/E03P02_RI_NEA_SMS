from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'E03P02_RI_SMS_NEA'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'model'), glob('model/*.urdf*') + glob('model/*.xacro')),
        (os.path.join('share', package_name, 'CAD'), glob('CAD/*.dxf')),  # Install CAD files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vboxsfms',
    maintainer_email='vboxsfms@todo.todo',
    description='RRP Robot CAD Trajectory Execution',
    license='TODO: License declaration',
    tests_require=['pytest'],
        entry_points={
        'console_scripts': [
            'dxf_parser = E03P02_RI_SMS_NEA.dxf_parser:main',
            'trajectory_planner = E03P02_RI_SMS_NEA.trajectory_planner:main',
            'inverse_kinematics = E03P02_RI_SMS_NEA.inverse_kinematics:main',
            'forward_kinematics = E03P02_RI_SMS_NEA.forward_kinematics:main',
            'joint_state_bridge = E03P02_RI_SMS_NEA.joint_state_bridge:main',
            'test_movement = E03P02_RI_SMS_NEA.test_movement:main',
            'path_publisher = E03P02_RI_SMS_NEA.path_publisher:main', 
        ],
    },
)