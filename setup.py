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
            'turtle_tf2_broadcaster = my_r2_tf2.turtle_tf2_broadcaster:main',
            'tf2_to_cmd_vel = my_r2_tf2.tf2_to_cmd_vel:main', 
            'get_cmd_vel = my_r2_tf2.get_cmd_vel:main',
        ],
    },
)
