from setuptools import find_packages, setup

package_name = 'explorer_dashboard'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # The marker file that lets "ros2 pkg list" and "ros2 run" find this package.
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    # threepp is the 3D engine. It comes from PyPI, not from a ROS package, so it is
    # named here and not in package.xml (rosdep does not know it).
    install_requires=['setuptools', 'threepp>=2026.10.11'],
    zip_safe=True,
    maintainer='Lars Ivar Hatledal',
    maintainer_email='larsivarhatledal@gmail.com',
    description='Mission control for the exploring robot: a 3D view of what the robot '
                'believes, and the operator\'s buttons.',
    license='MIT',
    entry_points={
        'console_scripts': [
            # "ros2 run explorer_dashboard dashboard" calls main() in dashboard_node.py.
            'dashboard = explorer_dashboard.dashboard_node:main',
        ],
    },
)
