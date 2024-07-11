from setuptools import setup
from glob import glob
import os

package_name                  = 'morphing_lander'
mpc_submodule                 = 'morphing_lander/mpc'
cvae_submodule                = 'morphing_lander/cvae'
data_utils_submodule          = 'morphing_lander/data_utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name,mpc_submodule,cvae_submodule,data_utils_submodule],
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
            'drive_controller_hardware = morphing_lander.drive_controller_hardware:main',
            'drive_controller_sim = morphing_lander.drive_controller_sim:main',
            'mpc_controller_hardware = morphing_lander.mpc_controller_hardware:main',
            'mpc_controller_sim = morphing_lander.mpc_controller_sim:main',
            'rl_controller_sim = morphing_lander.rl_controller_sim:main',
            'load_cell_test = morphing_lander.load_cell_test:main',
            'relay_mocap = morphing_lander.relay_mocap:main'
        ],
    },
)
