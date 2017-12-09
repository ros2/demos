from setuptools import find_packages
from setuptools import setup

package_name = 'topic_monitor'

setup(
    name=package_name,
    version='0.4.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'launch',
        'setuptools',
    ],
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
            'launch_depth_demo = topic_monitor.launch_files.launch_depth_demo:main',
            'launch_fragmentation_demo ='
                'topic_monitor.launch_files.launch_fragmentation_demo:main',
            'launch_reliability_demo ='
                'topic_monitor.launch_files.launch_reliability_demo:main',
        ],
    },
)
