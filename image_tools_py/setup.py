from setuptools import setup

package_name = 'image_tools_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    py_modules=[
        'burger_py',
        'cam2image_py',
        'showimage_py',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Chris Lalancette',
    author_email='clalancette@osrfoundation.org',
    maintainer='Chris Lalancette',
    maintainer_email='clalancette@osrfoundation.org',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Python tools to capture/play back images to/from DDS subscriptions/publications.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cam2image_py = cam2image_py:main',
            'showimage_py = showimage_py:main',
        ],
    },
)
