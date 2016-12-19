from setuptools import setup

setup(
    name='topic_monitor',
    version='0.0.0',
    packages=[],
    py_modules=[
        'topic_monitor.launch_files.launch_depth_demo',
        'topic_monitor.launch_files.launch_fragmentation_demo',
        'topic_monitor.launch_files.launch_reliability_demo',
        'topic_monitor.scripts.data_publisher',
        'topic_monitor.scripts.topic_monitor',
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
    entry_points={
        'console_scripts': [
            'topic_monitor_data_publisher = topic_monitor.scripts.data_publisher:main',
            'topic_monitor = topic_monitor.scripts.topic_monitor:main',
            'topic_monitor_launch_depth_demo = topic_monitor.launch_files.launch_depth_demo:main',
            'topic_monitor_launch_fragmentation_demo ='
                'topic_monitor.launch_files.launch_fragmentation_demo:main',
            'topic_monitor_launch_reliability_demo ='
                'topic_monitor.launch_files.launch_reliability_demo:main',
        ],
    },
)
