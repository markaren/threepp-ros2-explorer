from setuptools import find_packages, setup

package_name = 'explorer_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # These two entries are what makes `ros2 run` and `ros2 pkg list` find the package.
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lars Ivar Hatledal',
    maintainer_email='larsivarhatledal@gmail.com',
    description="The robot's brain: an occupancy grid mapper and a frontier explorer",
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        # `ros2 run explorer_nav mapper` starts explorer_nav/mapper_node.py:main, and so on.
        'console_scripts': [
            'mapper = explorer_nav.mapper_node:main',
            'explorer = explorer_nav.explorer_node:main',
        ],
    },
)
