from setuptools import setup, find_packages

setup(
    name="bosdyn_spot_navigation_graph_nav",
    version="0.1.0",
    packages=find_packages(include=["src", "src.*"]),
)