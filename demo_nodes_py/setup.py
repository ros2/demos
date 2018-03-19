from setuptools import find_packages
from setuptools import setup

package_name = 'demo_nodes_py'

setup(
    name=package_name,
    version='0.4.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Esteve Fernandez',
    author_email='esteve@osrfoundation.org',
    maintainer='Mikael Arguedas',
    maintainer_email='mikael@osrfoundation.org',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'Python nodes which were previously in the ros2/examples repository '
        'but are now just used for demo purposes.'
    ),
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'listener = demo_nodes_py.topics.listener:main',
            'talker = demo_nodes_py.topics.talker:main',
            'listener_qos = demo_nodes_py.topics.listener_qos:main',
            'talker_qos = demo_nodes_py.topics.talker_qos:main',
            'add_two_ints_client = demo_nodes_py.services.add_two_ints_client:main',
            'add_two_ints_client_async = demo_nodes_py.services.add_two_ints_client_async:main',
            'add_two_ints_server = demo_nodes_py.services.add_two_ints_server:main'
        ],
    },
)
