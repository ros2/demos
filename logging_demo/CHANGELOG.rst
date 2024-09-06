^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package logging_demo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.33.5 (2024-09-06)
-------------------

0.33.4 (2024-06-27)
-------------------

0.33.3 (2024-05-13)
-------------------

0.33.2 (2024-03-28)
-------------------
* Update maintainer list in package.xml files (`#665 <https://github.com/ros2/demos/issues/665>`_)
* Contributors: Michael Jeronimo

0.33.1 (2024-02-07)
-------------------

0.33.0 (2024-01-24)
-------------------
* Migrate std::bind calls to lambda expressions (`#659 <https://github.com/ros2/demos/issues/659>`_)
* Contributors: Felipe Gomes de Melo

0.32.1 (2023-12-26)
-------------------

0.32.0 (2023-11-06)
-------------------

0.31.1 (2023-09-07)
-------------------

0.31.0 (2023-08-21)
-------------------

0.30.1 (2023-07-11)
-------------------

0.30.0 (2023-06-12)
-------------------

0.29.0 (2023-06-07)
-------------------

0.28.1 (2023-05-11)
-------------------

0.28.0 (2023-04-27)
-------------------

0.27.0 (2023-04-13)
-------------------

0.26.0 (2023-04-11)
-------------------

0.25.0 (2023-03-01)
-------------------

0.24.1 (2023-02-24)
-------------------

0.24.0 (2023-02-14)
-------------------
* Update the demos to C++17. (`#594 <https://github.com/ros2/demos/issues/594>`_)
* [rolling] Update maintainers - 2022-11-07 (`#589 <https://github.com/ros2/demos/issues/589>`_)
* Contributors: Audrow Nash, Chris Lalancette

0.23.0 (2022-11-02)
-------------------

0.22.0 (2022-09-13)
-------------------
* Change dependency from 'rosidl_cmake' to 'rosidl_default_generators' (`#578 <https://github.com/ros2/demos/issues/578>`_)
* Contributors: Jacob Perron

0.21.0 (2022-04-29)
-------------------

0.20.1 (2022-04-08)
-------------------

0.20.0 (2022-03-01)
-------------------

0.19.0 (2022-01-14)
-------------------

0.18.0 (2021-12-17)
-------------------
* Update maintainers to Audrow Nash and Michael Jeronimo (`#543 <https://github.com/ros2/demos/issues/543>`_)
* Additional fixes for documentation in demos. (`#538 <https://github.com/ros2/demos/issues/538>`_)
* Contributors: Audrow Nash, Chris Lalancette

0.17.0 (2021-10-18)
-------------------

0.16.0 (2021-08-11)
-------------------
* Use rosidl_get_typesupport_target() (`#529 <https://github.com/ros2/demos/issues/529>`_)
* Contributors: Shane Loretz

0.15.0 (2021-05-14)
-------------------

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
* Update logging macros (`#476 <https://github.com/ros2/demos/issues/476>`_)
* Contributors: Audrow Nash

0.11.0 (2020-12-10)
-------------------
* Update the package.xml files with the latest Open Robotics maintainers (`#466 <https://github.com/ros2/demos/issues/466>`_)
* Contributors: Michael Jeronimo

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
