import os
from glob import glob

from setuptools import find_packages, setup

package_name = "autodrive_skku_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         [os.path.join("resource", package_name)]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Yulee3542",
    maintainer_email="yulee3542@gmail.com",
    description="autodrive_skku 차량 노드/미션 로직 (ROS 2)",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "arduino_node = autodrive_skku_ros.nodes.arduino_node:ros_main",
            "camera_node = autodrive_skku_ros.nodes.camera_node:ros_main",
            "lidar_node = autodrive_skku_ros.nodes.lidar_node:ros_main",
            "odometry_node = autodrive_skku_ros.nodes.odometry_node:ros_main",
            "mission_node = autodrive_skku_ros.nodes.mission_node:main",
            "teleop_node = autodrive_skku_ros.nodes.teleop_node:main",
        ],
    },
)
