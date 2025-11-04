from setuptools import setup
import os
from glob import glob

package_name = 'mein_turtlebot_paket'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch-Dateien hinzufügen
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ihr Name',
    maintainer_email='ihre.email@example.com',
    description='Einfaches Paket zur Steuerung des Turtlebot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtlebot_forward_node = mein_turtlebot_paket.turtlebot_forward_node:main',
            'colour_detection = mein_turtlebot_paket.colour_detection:main'
        ],
    },
)