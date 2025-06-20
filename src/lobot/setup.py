from setuptools import find_packages, setup

package_name = "lobot"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jay",
    maintainer_email="smitelims15@gmail.com",
    description="ROKEY BOOT CAMP Package",
    license="Apache 2.0 License",
    entry_points={
        "console_scripts": [
            "block_sort=lobot.task.block_sort:main",
        ],
    },
)
