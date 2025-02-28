from setuptools import setup
import os 
from glob import glob

package_name = 'kaist_rosbag'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name + '/config', ['config/record_topics.sh']),
        ('share/' + package_name + '/launch', ['launch/rosbag_trigger.launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='usrg',
    maintainer_email='jeinking@kaist.ac.kr',
    description='ROS 2 package for triggered bag recording',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rosbag_trigger = kaist_rosbag.rosbag_trigger:main'
        ],
    },
)
