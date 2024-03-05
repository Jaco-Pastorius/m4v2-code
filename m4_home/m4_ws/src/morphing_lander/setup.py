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
            'descent_controller = morphing_lander.descent_controller:main',
            'tilt_controller = morphing_lander.tilt_controller:main',
            'altitude_controller = morphing_lander.altitude_controller:main',
            'sim_controller_acados = morphing_lander.sim_controller_acados:main',
            'load_cell_test = morphing_lander.load_cell_test:main'
        ],
    },
)
