from setuptools import setup

package_name = "robot_ledger"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    # colcon test が pytest でテストを集めるための印（無いと unittest で探して 0 件になる）
    extras_require={"test": ["pytest"]},
    zip_safe=True,
    maintainer="Y.Fujita",
    description="ロボットの稼働を署名つきの台帳に記録し、使用範囲の中の仕事だけを数えて請求額を出す",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "ledger_recorder = robot_ledger.recorder_node:main",
            "ledger_verify = robot_ledger.cli:verify_main",
            "ledger_bill = robot_ledger.cli:bill_main",
            "inspection_recorder = robot_ledger.inspection_recorder_node:main",
        ],
    },
)
