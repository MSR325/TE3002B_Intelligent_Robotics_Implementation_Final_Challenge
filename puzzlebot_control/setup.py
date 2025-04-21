from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'puzzlebot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.[yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chrisrvt',
    maintainer_email='christianvillarrealt@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'open_loop_ctrl = puzzlebot_control.open_loop_ctrl:main',
            'square_path_ctrl = puzzlebot_control.square_path_ctrl:main',
            'path_generator = puzzlebot_control.path_generator:main',
            'open_loop_path_ctrl = puzzlebot_control.open_loop_path_ctrl:main',
            'segmented_path_generator = puzzlebot_control.segmented_path_generator:main',
            'fsm_open_loop_path_ctrl = puzzlebot_control.fsm_open_loop_path_ctrl:main',

        ],
    },
)
