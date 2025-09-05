from setuptools import find_packages, setup

package_name = 'capacitynet'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'talker=capacitynet.capacitynet:main',
        ],
    },
)


#  data_files=[
#         ('share/ament_index/resource_index/packages',
#             ['resource/' + package_name]),
#         ('share/' + package_name, ['package.xml']),
#         (os.path.join('share', package_name, 'launch'),                                      glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
#         (os.path.join('share', package_name, 'curobo_doosan/src/m1013'),                           glob(os.path.join('curobo_doosan/src/m1013', '*.*'))),
#         (os.path.join('share', package_name, 'curobo_doosan/src/m1013/meshes', 'm1013_white'),     glob(os.path.join('curobo_doosan/src/m1013/meshes/m1013_white', '*.dae*'))),
#         (os.path.join('share', package_name, 'curobo_doosan/src/m1013/meshes', 'm1013_collision'), glob(os.path.join('curobo_doosan/src/m1013/meshes/m1013_collision', '*.dae*'))),
#         (os.path.join('share', package_name, 'rviz'),                                        glob('rviz/*.rviz')),
#     ],

    # entry_points={
    #     'console_scripts': [
    #         'talker = curobo_ros.publisher_member_function:main',