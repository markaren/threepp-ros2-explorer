from setuptools import find_packages, setup

package_name = 'explorer_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # The marker file that lets `ros2 run` and `ros2 pkg list` find the package.
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lars Ivar Hatledal',
    maintainer_email='larsivarhatledal@gmail.com',
    description='Finds orange traffic cones in the camera image and places them on the map.',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            # `ros2 run explorer_vision cone_spotter` calls main() in cone_spotter_node.py
            'cone_spotter = explorer_vision.cone_spotter_node:main',
        ],
    },
)
