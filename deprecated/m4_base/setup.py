from setuptools import setup
from glob import glob
import os
package_name = 'm4_base'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='imandralis@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'offboard_control = m4_base.offboard_control:main',
            'tilt_controller = m4_base.tilt_controller:main',
            'drive_controller = m4_base.drive_controller:main',
            'angle_broadcaster = m4_base.angle_broadcaster:main',
            'relay_mocap = m4_base.relay_mocap:main',
        ],
    },
)
