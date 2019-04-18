^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package image_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
