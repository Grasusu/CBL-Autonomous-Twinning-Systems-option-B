from glob import glob
import os

from setuptools import setup


package_name = "tb3_pesticide_dt"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "docs"), glob("docs/*.md")),
        (os.path.join("share", package_name, "maps"), glob("maps/*")),
        (
            os.path.join("share", package_name, "models", "visual_twin_burger"),
            glob("models/visual_twin_burger/*"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="2IRR10 Team",
    maintainer_email="you@example.com",
    description="Autonomous pesticide inspection digital twin demo for TurtleBot3.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "plant_mission_node = tb3_pesticide_dt.plant_mission_node:main",
            "plant_nav2_mission_node = tb3_pesticide_dt.plant_nav2_mission_node:main",
            "inspection_twin_node = tb3_pesticide_dt.inspection_twin_node:main",
            "twin_safety_node = tb3_pesticide_dt.twin_safety_node:main",
            "arena_map_node = tb3_pesticide_dt.arena_map_node:main",
            "nav2_initial_pose_node = tb3_pesticide_dt.nav2_initial_pose_node:main",
            "gazebo_pose_mirror_node = tb3_pesticide_dt.gazebo_pose_mirror_node:main",
            "option_b_environment_node = tb3_pesticide_dt.option_b_environment_node:main",
        ],
    },
)
