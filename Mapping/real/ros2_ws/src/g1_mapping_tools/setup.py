from setuptools import find_packages, setup


package_name = "g1_mapping_tools"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/normalized_mapping.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="G1 Hackathon Team",
    maintainer_email="devnull@example.com",
    description="Diagnostics and artifact writers for G1 mapping.",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "sensor_doctor = g1_mapping_tools.sensor_doctor:main",
            "trajectory_writer = g1_mapping_tools.trajectory_writer:main",
            "mapping_adapter = g1_mapping_tools.mapping_adapter:main",
            "map_accumulator = g1_mapping_tools.map_accumulator:main",
            "zmq_camera_bridge = g1_mapping_tools.zmq_camera_bridge:main",
            "zmq_camera_probe = g1_mapping_tools.zmq_camera_probe:main",
        ]
    },
)
