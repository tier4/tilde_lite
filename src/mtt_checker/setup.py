from glob import glob
import os

from setuptools import setup

package_name = "mtt_checker"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob("~/launch/*_launch.py")),
        ("share/" + package_name, glob(os.path.join("launch", "*_launch.py"))),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="akm",
    maintainer_email="akm@isp.co.jp",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mtt_checker = mtt_checker.mtt_checker:main",
        ],
    },
)
