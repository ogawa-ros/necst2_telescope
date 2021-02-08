from setuptools import setup

package_name = "necst2_telescope"

scripts = [
    "soft_limit_1st",
    "soft_limit_2nd",
    "antenna_commander_pid",
    "coordinate_calc",
    "optical_pointing",
    "tpro01",
]  # reconsider when scripts' directory separated or its hierarchical structure changed.
executors = [
    "exec_antenna_commander",
]


setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="telescopio",
    maintainer_email="s_ta.matsumoto@p.s.osakafu-u.ac.jp",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [f"{name}={package_name}.{name}:main" for name in scripts]
        + [f"{name}={package_name}.{name}:main" for name in executors],
    },
)
