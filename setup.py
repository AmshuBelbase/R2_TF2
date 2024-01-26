from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_r2_tf2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='amshu',
    maintainer_email='amsubelbs@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [ 
        	'static_turtle_tf2_broadcaster = learning_tf2_py.static_turtle_tf2_broadcaster:main',
            'turtle_tf2_listener = learning_tf2_py.turtle_tf2_listener:main',
            'turtle_tf2_broadcaster = learning_tf2_py.turtle_tf2_broadcaster:main',
            'tf2_to_cmd_vel = learning_tf2_py.tf2_to_cmd_vel:main',
            'dynamic_pose_to_tf = learning_tf2_py.dynamic_pose_to_tf:main',
            'get_cmd_vel = learning_tf2_py.get_cmd_vel:main',
        ],
    },
)
