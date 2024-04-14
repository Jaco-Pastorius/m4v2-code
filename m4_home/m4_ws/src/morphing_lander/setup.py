from setuptools import setup
from glob import glob
import os
package_name = 'morphing_lander'

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
            'tilt_controller_hardware = morphing_lander.tilt_controller_hardware:main',
            'tilt_controller_sim = morphing_lander.tilt_controller_sim:main',
            'mpc_controller_hardware = morphing_lander.mpc_controller_hardware:main',
            'mpc_controller_sim = morphing_lander.mpc_controller_sim:main',
            'load_cell_test = morphing_lander.load_cell_test:main',
            'keyboard_publisher = morphing_lander.keyboard_publisher:main',
            'relay_mocap = morphing_lander.relay_mocap:main'
        ],
    },
)
