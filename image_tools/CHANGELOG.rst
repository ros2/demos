^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package image_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.20.4 (2024-05-15)
-------------------

0.20.3 (2023-01-10)
-------------------

0.20.2 (2022-05-10)
-------------------

0.20.1 (2022-04-08)
-------------------

0.20.0 (2022-03-01)
-------------------
* Install includes to include/${PROJECT_NAME} (`#548 <https://github.com/ros2/demos/issues/548>`_)
* Contributors: Shane Loretz

0.19.0 (2022-01-14)
-------------------
* Fix include order and relative paths for cpplint (`#551 <https://github.com/ros2/demos/issues/551>`_)
* Reduce the number of OpenCV libraries image_tools links against. (`#549 <https://github.com/ros2/demos/issues/549>`_)
* Adds copy constructor and assignment operator to ROSCvMatContainer (`#546 <https://github.com/ros2/demos/issues/546>`_)
* Contributors: Chris Lalancette, Gonzo, Jacob Perron

0.18.0 (2021-12-17)
-------------------
* Fixes for uncrustify 0.72 (`#545 <https://github.com/ros2/demos/issues/545>`_)
* Update maintainers to Audrow Nash and Michael Jeronimo (`#543 <https://github.com/ros2/demos/issues/543>`_)
* Additional fixes for documentation in demos. (`#538 <https://github.com/ros2/demos/issues/538>`_)
* Contributors: Audrow Nash, Chris Lalancette

0.17.0 (2021-10-18)
-------------------
* Fixing deprecated subscriber callback warnings (`#532 <https://github.com/ros2/demos/issues/532>`_)
* Contributors: Abrar Rahman Protyasha

0.16.0 (2021-08-11)
-------------------
* ambigulity: unknown type name 'nullptr_t' (`#528 <https://github.com/ros2/demos/issues/528>`_)
* Add type masquerading demos (`#482 <https://github.com/ros2/demos/issues/482>`_)
* Contributors: Audrow Nash, William Woodall, xwnb

0.15.0 (2021-05-14)
-------------------
* Add support for visualizing yuv422 (`#499 <https://github.com/ros2/demos/issues/499>`_)
* Contributors: joshua-qnx

0.14.2 (2021-04-26)
-------------------

0.14.1 (2021-04-19)
-------------------

0.14.0 (2021-04-06)
-------------------

0.13.0 (2021-03-25)
-------------------

0.12.1 (2021-03-18)
-------------------

0.12.0 (2021-01-25)
-------------------

0.11.0 (2020-12-10)
-------------------
* Initialize time stamp for published image messages (`#475 <https://github.com/ros2/demos/issues/475>`_)
* Update the package.xml files with the latest Open Robotics maintainers (`#466 <https://github.com/ros2/demos/issues/466>`_)
* Added more parameters for camera topic examples (`#465 <https://github.com/ros2/demos/issues/465>`_)
* Contributors: Jacob Perron, Michael Jeronimo

0.10.1 (2020-09-21)
-------------------

0.10.0 (2020-06-17)
-------------------

0.9.3 (2020-06-01)
------------------

0.9.2 (2020-05-26)
------------------

0.9.1 (2020-05-12)
------------------

0.9.0 (2020-04-30)
------------------
* Replace deprecated launch_ros usage (`#437 <https://github.com/ros2/demos/issues/437>`_)
* Fix frame_id  (`#433 <https://github.com/ros2/demos/issues/433>`_)
* Update launch_ros action usage (`#431 <https://github.com/ros2/demos/issues/431>`_)
* code style only: wrap after open parenthesis if not in one line (`#429 <https://github.com/ros2/demos/issues/429>`_)
* Contributors: Dirk Thomas, Gonzo, Jacob Perron

0.8.4 (2019-11-19)
------------------

0.8.3 (2019-11-11)
------------------
* image_tools should start with reliable policy (`#411 <https://github.com/ros2/demos/issues/411>`_)
* Contributors: Shane Loretz

0.8.2 (2019-11-08)
------------------

0.8.1 (2019-10-23)
------------------
* Fix burguer mode parameter typo in help text (`#406 <https://github.com/ros2/demos/issues/406>`_)
* Replace ready_fn with ReadyToTest action (`#404 <https://github.com/ros2/demos/issues/404>`_)
* [image_tools] Use ROS parameters instead of regular CLI arguments (`#398 <https://github.com/ros2/demos/issues/398>`_)
* Contributors: Brian Marchi, Jacob Perron, Peter Baughman

0.8.0 (2019-09-26)
------------------
* Adding visibility macros to demos (`#381 <https://github.com/ros2/demos/issues/381>`_)
* Demos using composition (`#375 <https://github.com/ros2/demos/issues/375>`_)
* Contributors: Siddharth Kucheria

0.7.6 (2019-05-30)
------------------

0.7.5 (2019-05-29)
------------------
* Remove debugging prints from showimage. (`#358 <https://github.com/ros2/demos/issues/358>`_)
* Contributors: Chris Lalancette

0.7.4 (2019-05-20)
------------------

0.7.3 (2019-05-10)
------------------

0.7.2 (2019-05-08)
------------------
* changes to avoid deprecated API's (`#332 <https://github.com/ros2/demos/issues/332>`_)
* Corrected publish calls with shared_ptr signature (`#327 <https://github.com/ros2/demos/issues/327>`_)
* Migrate launch tests to new launch_testing features & API (`#318 <https://github.com/ros2/demos/issues/318>`_)
* Contributors: Michel Hidalgo, William Woodall, ivanpauno

0.7.1 (2019-04-26)
------------------
* Removed support for OpenCV 2. (`#322 <https://github.com/ros2/demos/issues/322>`_)
* Contributors: Jacob Perron

0.7.0 (2019-04-14)
------------------
* Added launch along with launch_testing as test dependencies. (`#313 <https://github.com/ros2/demos/issues/313>`_)
* Contributors: Michel Hidalgo

0.6.2 (2019-01-15)
------------------
* Updated to support OpenCV 2, 3 and 4 (`#307 <https://github.com/ros2/demos/issues/307>`_)
* Updated for OpenCV v4.0 compatibility (`#306 <https://github.com/ros2/demos/issues/306>`_)
* Updated to show freq parameter on help only when necessary (`#296 <https://github.com/ros2/demos/issues/296>`_)
* Contributors: Gonzo, Jacob Perron

0.6.1 (2018-12-13)
------------------

0.6.0 (2018-12-07)
------------------
* Updated to prevent frame going out of scope when converting RGB -> BGR (`#288 <https://github.com/ros2/demos/issues/288>`_)
* Added semicolons to all RCLCPP and RCUTILS macros. (`#278 <https://github.com/ros2/demos/issues/278>`_)
* Updated to keep only the last sample in the image tools by default. (`#238 <https://github.com/ros2/demos/issues/238>`_)
* Contributors: Chris Lalancette, sgvandijk

0.5.1 (2018-06-28)
------------------

0.5.0 (2018-06-27)
------------------
* Fixed linting errors in ``burger.cpp`` (`#260 <https://github.com/ros2/demos/issues/260>`_)
  * Signed-off-by: Chris Lalancette <clalancette@openrobotics.org>
* Fixed a bug that occurred when the resolution was less than the bugger. (`#258 <https://github.com/ros2/demos/issues/258>`_)
* Updated launch files to account for the "old launch" getting renamespaced as ``launch`` -> ``launch.legacy``. (`#239 <https://github.com/ros2/demos/issues/239>`_)
* Contributors: Chris Lalancette, Dirk Thomas, William Woodall
