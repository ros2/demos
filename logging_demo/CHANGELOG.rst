^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package logging_demo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Set logging format in tests to avoid drift when defaults change. (`#420 <https://github.com/ros2/demos/issues/420>`_)
* Contributors: Steven! Ragnarök

0.8.4 (2019-11-19)
------------------

0.8.3 (2019-11-11)
------------------

0.8.2 (2019-11-08)
------------------

0.8.1 (2019-10-23)
------------------
* Replace ready_fn with ReadyToTest action (`#404 <https://github.com/ros2/demos/issues/404>`_)
* Fix logging demo test (`#400 <https://github.com/ros2/demos/issues/400>`_)
* Contributors: Michel Hidalgo, Peter Baughman

0.8.0 (2019-09-26)
------------------

0.7.6 (2019-05-30)
------------------

0.7.5 (2019-05-29)
------------------

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
* Updated package maintainer for logging_demo and topic_monitor (`#285 <https://github.com/ros2/demos/issues/285>`_)
* Updated to use new error handling API from rcutils (`#284 <https://github.com/ros2/demos/issues/284>`_)
* Added semicolons to all RCLCPP and RCUTILS macros. (`#278 <https://github.com/ros2/demos/issues/278>`_)
* Updated to use add_compile_options instead of setting only cxx flags
* Contributors: Chris Lalancette, Mikael Arguedas, Scott K Logan, William Woodall

0.5.1 (2018-06-28)
------------------

0.5.0 (2018-06-27)
------------------
* Updated to use the log level parsing functions from ``rcutils``. (`#240 <https://github.com/ros2/demos/issues/240>`_)
* Updated launch files to account for the "old launch" getting renamespaced as ``launch`` -> ``launch.legacy``. (`#239 <https://github.com/ros2/demos/issues/239>`_)
* Updated to avoid use of newly deprecated class loader headers. (`#229 <https://github.com/ros2/demos/issues/229>`_)
* Contributors: Dirk Thomas, Michael Carroll, Mikael Arguedas, William Woodall, dhood
