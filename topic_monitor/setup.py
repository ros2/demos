from ament_python.data_files import get_data_files
from ament_python.script_dir import install_scripts_to_libexec
from setuptools import setup

package_name = 'topic_monitor'
data_files = get_data_files(package_name)
install_scripts_to_libexec(package_name)

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
    data_files=data_files,
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
