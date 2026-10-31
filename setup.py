from glob import glob

from setuptools import setup

package_name = "pai_robot"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/worlds", glob("worlds/*.sdf")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="pai_student",
    maintainer_email="pai_student@example.com",
    description="PAI final assignment: LLM natural-language commander for a Gazebo diff-drive robot",
    license="MIT",
    entry_points={
        "console_scripts": [
            "llm_commander = pai_robot.llm_commander:main",
        ],
    },
)
