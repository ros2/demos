import glob
import os

from setuptools import find_packages
from setuptools import setup

package_name = 'topic_monitor'

setup(
    name=package_name,
    version='0.33.3',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/topic_monitor/launch',
            glob.glob(os.path.join('launch', '*_launch.py'))),
    ],
    install_requires=[
        'launch',
        'setuptools',
    ],
    zip_safe=True,
    maintainer='Aditya Pande, Audrow Nash, Michael Jeronimo',
    maintainer_email='aditya.pande@openrobotics.org, audrow@openrobotics.org, michael.jeronimo@openrobotics.org',  # noqa: E501
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Package containing tools for monitoring ROS 2 topics.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'data_publisher = topic_monitor.scripts.data_publisher:main',
            'topic_monitor = topic_monitor.scripts.topic_monitor:main',
        ],
    },
)
