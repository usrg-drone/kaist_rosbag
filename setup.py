from setuptools import setup
from glob import glob

package_name = 'kros'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        ('share/' + package_name,
            ['package.xml']),

        ('share/' + package_name + '/launch',
            glob('launch/*.launch.py')),

        ('share/' + package_name + '/config',
            glob('config/*')),
    ],

    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,

    maintainer='Hyunun Cho',
    maintainer_email='hyununcho@kaist.ac.kr',

    description='kros rosbag recorder',
    license='Apache-2.0',

    entry_points={
        'console_scripts': [
            'kros2 = kros.kros2:main',
        ],
    },
)
