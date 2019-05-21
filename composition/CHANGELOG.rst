^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package composition
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Renamed launch file, updated to avoid redundant default actions, and set output to screen. (`#326 <https://github.com/ros2/demos/issues/326>`_)
* Updated constructor to const ref to NodeOptions. (`#323 <https://github.com/ros2/demos/issues/323>`_)
* Added basic composition launch demo. (`#324 <https://github.com/ros2/demos/issues/324>`_)
* Contributors: Michael Carroll, William Woodall

0.7.0 (2019-04-14)
------------------
* Updated for new rclcpp_components package. (`#319 <https://github.com/ros2/demos/issues/319>`_)
* Added launch along with launch_testing as test dependencies. (`#313 <https://github.com/ros2/demos/issues/313>`_)
* Dropped legacy launch API usage. (`#311 <https://github.com/ros2/demos/issues/311>`_)
* Contributors: Michael Carroll, Michel Hidalgo

0.6.2 (2019-01-15)
------------------

0.6.1 (2018-12-13)
------------------

0.6.0 (2018-12-07)
------------------
* Added semicolons to all RCLCPP and RCUTILS macros. (`#278 <https://github.com/ros2/demos/issues/278>`_)
* Contributors: Chris Lalancette

0.5.1 (2018-06-28)
------------------

0.5.0 (2018-06-27)
------------------
* Updated launch files to account for the "old launch" getting renamespaced as ``launch`` -> ``launch.legacy``. (`#239 <https://github.com/ros2/demos/issues/239>`_)
* Updated to avoid newly deprecated class loader headers. (`#229 <https://github.com/ros2/demos/issues/229>`_)
* Added a periodic log message while waiting for a service response.
* Contributors: Dirk Thomas, Michael Carroll, Mikael Arguedas, William Woodall
