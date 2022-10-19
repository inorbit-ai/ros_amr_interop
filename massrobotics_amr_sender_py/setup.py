from setuptools import setup, find_packages
import os
from glob import glob


package_name = "massrobotics_amr_sender"

share_dir = os.path.join("share", package_name)

setup(
    name=package_name,
    packages=find_packages(),
    package_data={"": ["schema.json"]},
    include_package_data=True,
    version="1.1.0",
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (share_dir, ["package.xml"]),
        # Include launch files
        (share_dir, glob("launch/*.launch.py")),
        # Sample config files
        (
            os.path.join(share_dir, "params"),
            [os.path.join("params", "sample_config.yaml")],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="InOrbit",
    maintainer_email="support@inorbit.ai",
    description="ROS2 node implementing a MassRobotics AMR Interoperability Sender",
    license="3-Clause BSD License",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "massrobotics_amr_node = massrobotics_amr_sender.massrobotics_amr_node:main"
        ],
    },
)
