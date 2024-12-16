from setuptools import find_packages, setup
from glob import glob
import os

package_name = "simple_arm_demo"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.py")),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*.yaml")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="sasm",
    maintainer_email="sasilva1998@gmail.com",
    description="TODO: Package description",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "arm_prm_demo = simple_arm_demo.arm_prm_demo:main",
            "arm_rrt_demo = simple_arm_demo.arm_rrt_demo:main",
            "arm_ompl_demo = simple_arm_demo.arm_ompl_demo:main",
        ],
    },
)
