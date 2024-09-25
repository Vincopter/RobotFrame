import os
import glob
from setuptools import find_packages
from setuptools import setup

package_name = 'robot_frame'

if __name__ == '__main__':

    setup(
        name=package_name,
        version='1.0.0',
        packages=[package_name, find_packages(exclude=['interactive', 'test'])],
        data_files=[
            ('share/' + package_name, ['package.xml']),
            (os.path.join('share', package_name), glob.glob('launchers/*.py')),
            (os.path.join('share', package_name), glob.glob('sources/*.py'))
            (os.path.join('share', package_name), glob.glob('scripts/*.sh'))
        ],
        install_requires=['setuptools'],
        zip_safe=True,
        description='The package summarizes the results of the final course assignment.',
        license='Apache License, Version 2.0',
        entry_points={
            'console_scripts': [
                'demo = sources.demonstration.py:main',
                'obstacle = sources.obstacle_controller.py:main',
            ],
        },
    )
