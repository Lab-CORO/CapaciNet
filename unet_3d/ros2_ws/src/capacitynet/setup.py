from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'capacitynet'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'reachability_node=capacitynet.rm_model.capacitynet:main',
            'workspace_reachability_node=capacitynet.control.nodes.workspace_reachability_node:main',
            'gradient_base_controller=capacitynet.control.nodes.gradient_base_controller:main',
            'workspace_probe=capacitynet.control.nodes.workspace_probe:main',
            'grasp=capacitynet.control.nodes.grasp:main',
            'replay_maps=capacitynet.control.nodes.replay_maps:main',
            'metrics_recorder=capacitynet.rm_model.metrics_recorder:main',
        ],
    },
)

