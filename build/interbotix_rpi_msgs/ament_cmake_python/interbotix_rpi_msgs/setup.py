from setuptools import find_packages
from setuptools import setup

setup(
    name='interbotix_rpi_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('interbotix_rpi_msgs', 'interbotix_rpi_msgs.*')),
)
