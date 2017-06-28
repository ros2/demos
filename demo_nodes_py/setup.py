from ament_python.data_files import get_data_files
from ament_python.script_dir import install_scripts_to_libexec
from setuptools import setup

package_name = 'demo_nodes_py'
data_files = get_data_files(package_name)
install_scripts_to_libexec(package_name)

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    py_modules=[
        'topics.listener', 'topics.talker',
        'topics.listener_qos', 'topics.talker_qos',
        'services.add_two_ints_client', 'services.add_two_ints_client_async',
        'services.add_two_ints_server'],
    data_files=data_files,
    install_requires=['setuptools'],
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
    test_suite='test',
    entry_points={
        'console_scripts': [
            'listener = topics.listener:main',
            'talker = topics.talker:main',
            'listener_qos = topics.listener_qos:main',
            'talker_qos = topics.talker_qos:main',
            'add_two_ints_client = services.add_two_ints_client:main',
            'add_two_ints_client_async = services.add_two_ints_client_async:main',
            'add_two_ints_server = services.add_two_ints_server:main'
        ],
    },
)
