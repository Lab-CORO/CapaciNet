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
            'reachability_node=capacitynet.capacitynet:main',
            'workspace_reachability_node=capacitynet.workspace_reachability_node:main',
            'gradient_controller_mock=capacitynet.gradient_controller_mock:main',
            'gradient_base_controller=capacitynet.gradient_base_controller:main',
            'workspace_probe=capacitynet.workspace_probe:main',
            'brain_orchestrator=capacitynet.brain_orchestrator:main',
            'brain=capacitynet.brain:main',
            'grasp=capacitynet.grasp:main',
            'test_attach_detach=capacitynet.test_attach_detach:main',
            'replay_maps=capacitynet.replay_maps:main',
            'metrics_recorder=capacitynet.metrics_recorder:main',
            'plot_metrics=capacitynet.plot_metrics:main',
        ],
    },
)

