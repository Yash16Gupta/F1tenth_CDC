from setuptools import setup
import os
from glob import glob

package_name = 'f1tenth_stack'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hongrui Zheng',
    maintainer_email='billyzheng.bz@gmail.com',
    description='Onboard drivers for vesc and sensors for F1TENTH vehicles.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'throttle_interpolator = f1tenth_stack.throttle_interpolator:main',
            'tf_publisher = f1tenth_stack.tf_publisher:main',
            'alpp = f1tenth_stack.alpp:main',
            'raceline = f1tenth_stack.raceline:main',
            'gap_follower = f1tenth_stack.gap_follower:main',
            'ftg_ema = f1tenth_stack.ftg_ema:main',
	        'obs = f1tenth_stack.obstacle_tester:main',
            'mux = f1tenth_stack.mux:main',
            'tmux = f1tenth_stack.tmux:main'
        ],
    },
)
