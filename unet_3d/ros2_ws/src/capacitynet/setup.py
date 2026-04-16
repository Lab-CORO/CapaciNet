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
            'gradient_controller_mock=capacitynet.gradient_controller_mock:main',
            'brain_orchestrator=capacitynet.brain_orchestrator:main',
        ],
    },
)

