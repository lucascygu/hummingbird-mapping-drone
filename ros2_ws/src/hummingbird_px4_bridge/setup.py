# Generated / assisted by Grok
from setuptools import find_packages, setup

package_name = "hummingbird_px4_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/px4_bridge.launch.py"]),
        ("share/" + package_name + "/config", ["config/px4_bridge.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Lucas Changyao Gu",
    maintainer_email="113392627+lucascygu@users.noreply.github.com",
    description="FAST-LIO2 odometry -> PX4 VehicleOdometry (external vision). Never arms.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "odom_bridge = hummingbird_px4_bridge.odom_bridge_node:main",
        ],
    },
)
