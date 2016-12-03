from setuptools import setup

setup(
    name='topic_monitor',
    version='0.0.0',
    packages=[],
    py_modules=[
        'data_publisher',
        'launch_data_publisher',
        'topic_monitor',
    ],
    install_requires=[
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
            'topic_monitor_data_publisher = data_publisher:main',
            'topic_monitor_launch_data_publisher = launch_data_publisher:main',
            'topic_monitor = topic_monitor:main',
        ],
    },
)
