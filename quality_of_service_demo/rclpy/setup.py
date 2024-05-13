from setuptools import setup

package_name = 'quality_of_service_demo_py'

setup(
    name=package_name,
    version='0.33.3',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Emerson Knapp',
    maintainer='Aditya Pande, Audrow Nash, Michael Jeronimo',
    maintainer_email='aditya.pande@openrobotics.org, audrow@openrobotics.org, michael.jeronimo@openrobotics.org',  # noqa: E501
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Python nodes to demonstrate ROS 2 QoS policies.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'deadline = quality_of_service_demo_py.deadline:main',
            'incompatible_qos = quality_of_service_demo_py.incompatible_qos:main',
            'lifespan = quality_of_service_demo_py.lifespan:main',
            'liveliness = quality_of_service_demo_py.liveliness:main',
            'message_lost_listener = quality_of_service_demo_py.message_lost_listener:main',
            'qos_overrides_listener = quality_of_service_demo_py.qos_overrides_listener:main',
            'qos_overrides_talker = quality_of_service_demo_py.qos_overrides_talker:main',
        ],
    },
)
