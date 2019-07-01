import os

from setuptools import setup
from setuptools.command.install_scripts import install_scripts

package_name = 'image_tools_py'


# This configuration deserves some explanation.  The goal is to generate a
# test file called something like 'test_showimage_cam2image__rmw_fastrtps_.py'
# that contains the test from test_showimage_cam2image.py.in, customized
# for the rmw_implementation we want to test and for absolute paths of this
# installation.  To do that, we override the 'install_scripts' stage of
# setuptools, since that is the stage that knows the paths to the 'cam2image_py'
# and 'showimage_py' executables.
#
# TODO(clalancette): One downside to what is being done below is that the output
# file is being left in the source directory.  That's currently necessary
# because the pytest invocation only looks there for test scripts.  It would
# be nicer to generate the output file in the 'build' directory and then point
# pytest at it, but I don't know how to do that.
class custom_install_scripts(install_scripts):

    def run(self):
        rmw_implementations = ['rmw_fastrtps_cpp']

        for rmw_impl in rmw_implementations:
            substs = {
                '@rmw_implementation@': rmw_impl,
                '@showimage_py_exe@': os.path.join(self.install_dir, 'showimage_py'),
                '@showimage_py_outfile@': os.path.realpath(os.path.join('test', 'showimage_py')),
                '@cam2image_py_exe@': os.path.join(self.install_dir, 'cam2image_py'),
                '@cam2image_py_outfile@': os.path.realpath(os.path.join('test', 'cam2image_py')),
            }

            infile = os.path.join('test', 'test_showimage_cam2image.py.in')
            outfile = os.path.join('test', 'test_showimage_cam2image__%s_.py' % (rmw_impl))
            with open(infile, 'r') as infp:
                with open(outfile, 'w') as outfp:
                    for line in infp:
                        for subst in substs:
                            line = line.replace(subst, "r'%s'  # noqa" % (substs[subst]))
                        outfp.write(line)

        install_scripts.run(self)


setup(
    name=package_name,
    version='0.1.0',
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
    author_email='clalancette@openrobotics.org',
    maintainer='Chris Lalancette',
    maintainer_email='clalancette@openrobotics.org',
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
    cmdclass={'install_scripts': custom_install_scripts},
)
