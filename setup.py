from glob import glob

from setuptools import setup

package_name = "kaist_rosbag"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="usrg",
    maintainer_email="donghun.han@kaist.ac.kr",
    description="ROS 2 package for triggered bag recording",
    license="TODO: License declaration",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "rosbag_recorder = kaist_rosbag.rosbag_recorder:main",
        ],
    },
)
