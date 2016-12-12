from setuptools import setup

setup(
    name='topic_monitor',
    version='0.0.0',
    packages=[],
    py_modules=[
        'launch_files.launch_data_publishers',
        'launch_files.launch_depth_demo',
        'launch_files.launch_durability_demo',
        'launch_files.launch_fragmentation_demo',
        'launch_files.launch_restarting_data_publisher',
        'scripts.data_publisher',
        'scripts.topic_monitor',
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
            'topic_monitor_data_publisher = scripts.data_publisher:main',
            'topic_monitor = scripts.topic_monitor:main',
            'topic_monitor_launch_data_publishers = launch_files.launch_data_publishers:main',
            'topic_monitor_launch_depth_demo = launch_files.launch_depth_demo:main',
            'topic_monitor_launch_durability_demo = launch_files.launch_durability_demo:main',
            'topic_monitor_launch_fragmentation_demo = launch_files.launch_fragmentation_demo:main',
            'topic_monitor_launch_restarting_data_publisher = launch_files.launch_restarting_data_publisher:main',
        ],
    },
)
