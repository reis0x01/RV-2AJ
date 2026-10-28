from glob import glob

from setuptools import find_packages, setup

package_name = "rv2aj_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml") + glob("config/*.json") + glob("config/*.txt")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Mohamed Ossama ELIWA",
    maintainer_email="mehmet.robotik.tr@gmail.com",
    description="RV-2AJ / CR1-571 control stack: driver, MoveIt planner, voice, translator, sequencer, GUI.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "driver = rv2aj_control.driver_node:main",
            "voice = rv2aj_control.voice_node:main",
            "translator = rv2aj_control.translator_node:main",
            "sequencer = rv2aj_control.sequencer_node:main",
            "gui = rv2aj_control.gui_node:main",
            "planner = rv2aj_control.planner_node:main",
            "plc_bridge = rv2aj_control.plc_bridge_node:main",
        ],
    },
)
