from setuptools import setup

package_name = 'action_tutorials_py'

setup(
    name=package_name,
    version='0.20.5',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Jacob Perron',
    author_email='jacob@openrobotics.org',
    maintainer='Audrow Nash, Michael Jeronimo',
    maintainer_email='audrow@openrobotics.org, michael.jeronimo@openrobotics.org',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Python action tutorials code.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fibonacci_action_server = action_tutorials_py.fibonacci_action_server:main',
            'fibonacci_action_client = action_tutorials_py.fibonacci_action_client:main',
        ],
    },
)
