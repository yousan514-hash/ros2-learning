from glob import glob

from setuptools import setup

package_name = "py_pubsub"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Y.Fujita",
    description="ROS 2 チュートリアル: Python でパブリッシャーとサブスクライバーを書く",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "talker = py_pubsub.publisher_member_function:main",
            "listener = py_pubsub.subscriber_member_function:main",
            "add_server = py_pubsub.add_two_ints_server:main",
            "add_client = py_pubsub.add_two_ints_client:main",
            "param_talker = py_pubsub.param_talker:main",
            "frame_broadcaster = py_pubsub.frame_broadcaster:main",
            "frame_listener = py_pubsub.frame_listener:main",
            "bag_recorder = py_pubsub.bag_recorder:main",
            "fibonacci_server = py_pubsub.fibonacci_server:main",
            "fibonacci_client = py_pubsub.fibonacci_client:main",
            "turtle_draw = py_pubsub.turtle_draw:main",
        ],
    },
)
