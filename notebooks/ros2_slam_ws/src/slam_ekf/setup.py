from setuptools import setup
import os
from glob import glob

package_name = 'slam_ekf'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='EKF state estimation node for a differential drive robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ekf_node = slam_ekf.ekf_node:main',
        ],
    },
)
