from setuptools import find_packages, setup

package_name = "hummingbird_task_agent"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/mission_gate.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Lucas Changyao Gu",
    maintainer_email="113392627+lucascygu@users.noreply.github.com",
    description="Hummingbird task-layer mission gate (high-level setpoints only).",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mission_gate = hummingbird_task_agent.mission_gate_node:main",
        ],
    },
)
