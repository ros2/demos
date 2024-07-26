^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package composition
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.20.5 (2024-07-26)
-------------------

0.20.4 (2024-05-15)
-------------------
* [composition] add launch action console output in the verify section (`#677 <https://github.com/ros2/demos/issues/677>`_) (`#682 <https://github.com/ros2/demos/issues/682>`_)
* Contributors: mergify[bot]

0.20.3 (2023-01-10)
-------------------

0.20.2 (2022-05-10)
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
* Fixing deprecated subscriber callback warnings (`#532 <https://github.com/ros2/demos/issues/532>`_)
* Contributors: Abrar Rahman Protyasha

0.16.0 (2021-08-11)
-------------------

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
* Fix leak(`#480 <https://github.com/ros2/demos/issues/480>`_) (`#481 <https://github.com/ros2/demos/issues/481>`_)
* Contributors: y-okumura-isp

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
* Replace deprecated launch_ros usage (`#437 <https://github.com/ros2/demos/issues/437>`_)
* Update launch_ros action usage (`#431 <https://github.com/ros2/demos/issues/431>`_)
* code style only: wrap after open parenthesis if not in one line (`#429 <https://github.com/ros2/demos/issues/429>`_)
* Contributors: Dirk Thomas, Jacob Perron

0.8.4 (2019-11-19)
------------------

0.8.3 (2019-11-11)
------------------

0.8.2 (2019-11-08)
------------------

0.8.1 (2019-10-23)
------------------
* Replace ready_fn with ReadyToTest action (`#404 <https://github.com/ros2/demos/issues/404>`_)
* Contributors: Peter Baughman

0.8.0 (2019-09-26)
------------------
* Add an demo component not inherited from rclcpp::Node (`#393 <https://github.com/ros2/demos/issues/393>`_)
* Contributors: Michael Carroll

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
