import os
import glob
from setuptools import find_packages, setup

package_name = 'my_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', '*.launch.py'))),
        ('share/' + package_name + '/model', glob.glob(os.path.join('model', '*.pt'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ggori',
    maintainer_email='ggori@todo.todo',
    description='TurtleBot3 A* path planning + Pure Pursuit driving with YOLOv8 emergency stop',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'integrated_navigation = my_bot.integrated_navigation:main',
        ],
    },
)
