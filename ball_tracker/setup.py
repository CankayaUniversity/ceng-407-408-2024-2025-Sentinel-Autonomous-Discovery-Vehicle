from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'ball_tracker'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
            ('share/ament_index/resource_index/packages',
                ['resource/' + package_name]),
            ('share/' + package_name, ['package.xml']),
            (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
            (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*.*")),
        ),
        ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ilteris',
    maintainer_email='ilteris.samur@outlook.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detect_ball = ball_tracker.detect_ball:main',
            'detect_ball_3d = ball_tracker.detect_ball_3d:main',
            'follow_ball = ball_tracker.follow_ball:main',
        ],
    },
)
