import os
from glob import glob

from setuptools import setup


package_name = "tag_relocalization"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="gwontaeyang",
    maintainer_email="gwontaeyang@users.noreply.github.com",
    description="Relocalize the robot from observed AprilTags and a generated tag map.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "tag_relocalization_node = tag_relocalization.tag_relocalization_node:main",
        ],
    },
)
