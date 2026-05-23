import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'task2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='puhster',
    maintainer_email='jure@puh.si',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'detect_faces = task2.detect_faces.main:main',
            'detect_rings = task2.detect_rings.main:main',
            'classify_face = task2.classify_face.main:main',
            'movement = task2.movement.main:main',
            'speak = task2.speak.main:main',
            'map_query = task2.map_query.main:main',
            'detect_anomalies = task2.anomaly_detection.main:main',
            'blue_line = task2.blue_line.main:main',
        ],
    },
)
