from setuptools import find_packages, setup

package_name = 'wheel_leg_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='qijingbo',
    maintainer_email='qihan9701@gmail.com',
    description='Wheel PI, VMC and LQR balance controller over ROS2 topics',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'wheel_leg_controller = wheel_leg_controller.controller_node:main',
        ],
    },
)
