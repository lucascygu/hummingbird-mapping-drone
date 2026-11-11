# Generated / assisted by Grok
from glob import glob

from setuptools import find_packages, setup

package_name = "hummingbird_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
        ("share/" + package_name + "/scripts", glob("scripts/*.sh")),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="Lucas Changyao Gu",
    maintainer_email="113392627+lucascygu@users.noreply.github.com",
    description="Hummingbird bringup: Mid-360 + FAST-LIO2 + PX4 bridge launch, configs and map tools.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "map_recorder = hummingbird_bringup.map_recorder:main",
            "map_tiler = hummingbird_bringup.map_tiler:main",
            "mid360_sim = hummingbird_bringup.mid360_sim:main",
            "sim_check = hummingbird_bringup.sim_check:main",
            "pcd_export = hummingbird_bringup.pcd_export:main",
        ],
    },
)
