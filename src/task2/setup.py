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
            'detect_faces = task2.detect_faces:main',
            'detect_rings = task2.detect_rings3:main',
            'classify_face = task2.classify_face:main',
        ],
    },
)
