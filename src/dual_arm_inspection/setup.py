from glob import glob

from setuptools import setup

package_name = "dual_arm_inspection"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/urdf", glob("urdf/*.xacro")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
        ("share/" + package_name + "/config", glob("config/*.json")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Y.Fujita",
    description="双腕の外観検査セル: 持つ腕と見る腕の協調、疑似カメラ、3 段判定（OK／NG／保留）",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "cell_node = dual_arm_inspection.cell_node:main",
            "camera_node = dual_arm_inspection.camera_node:main",
            "inspector_node = dual_arm_inspection.inspector_node:main",
        ],
    },
)
