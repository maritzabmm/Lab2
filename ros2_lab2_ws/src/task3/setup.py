import os 
from glob import glob
from setuptools import find_packages, setup

package_name = 'task3'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][y|a]ml]'))),
        (os.path.join('share', package_name, 'img_data'), glob(os.path.join('img_data', '*.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adoolph',
    maintainer_email='adoolph@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tf_broadcaster = task3.tf_broadcaster:main',
            'static_robot_lidar = task3.static_robot_lidar:main',
            'broadcast_robot_scanner = task3.broadcast_robot_scanner:main',
            'tf_listener = task3.tf_listener:main',
            'tf_robot_rev_listener = task3.tf_robot_rev_listener:main',        
        ],
    },
)
