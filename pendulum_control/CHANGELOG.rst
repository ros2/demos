^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pendulum_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* [pendulum_control Install targets to project lib (`#624 <https://github.com/ros2/demos/issues/624>`_)
* Contributors: Yadu

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

0.21.0 (2022-04-29)
-------------------

0.20.1 (2022-04-08)
-------------------

0.20.0 (2022-03-01)
-------------------

0.19.0 (2022-01-14)
-------------------
* Fix include order and relative paths for cpplint (`#551 <https://github.com/ros2/demos/issues/551>`_)
* Contributors: Jacob Perron

0.18.0 (2021-12-17)
-------------------
* Remove the malloc_hook from the pendulum_demo. (`#544 <https://github.com/ros2/demos/issues/544>`_)
* Update maintainers to Audrow Nash and Michael Jeronimo (`#543 <https://github.com/ros2/demos/issues/543>`_)
* Additional fixes for documentation in demos. (`#538 <https://github.com/ros2/demos/issues/538>`_)
* Fix documentation for pendulum_control. (`#537 <https://github.com/ros2/demos/issues/537>`_)
* Contributors: Audrow Nash, Chris Lalancette

0.17.0 (2021-10-18)
-------------------

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
* Replace rmw_connext_cpp with rmw_connextdds (`#489 <https://github.com/ros2/demos/issues/489>`_)
* Contributors: Andrea Sorbini

0.12.1 (2021-03-18)
-------------------

0.12.0 (2021-01-25)
-------------------
* Remove ineffective log output (`#450 <https://github.com/ros2/demos/issues/450>`_) (`#477 <https://github.com/ros2/demos/issues/477>`_)
* Contributors: y-okumura-isp

0.11.0 (2020-12-10)
-------------------
* Update the package.xml files with the latest Open Robotics maintainers (`#466 <https://github.com/ros2/demos/issues/466>`_)
* Contributors: Michael Jeronimo

0.10.1 (2020-09-21)
-------------------
* Remove deprecated warning (`#459 <https://github.com/ros2/demos/issues/459>`_)
* Contributors: Anas Abou Allaban

0.10.0 (2020-06-17)
-------------------
* Follow API/file name changes (`ros2/realtime_support#94 <https://github.com/ros2/realtime_support/issues/94>`_) (`#451 <https://github.com/ros2/demos/issues/451>`_)
* Contributors: y-okumura-isp

0.9.3 (2020-06-01)
------------------

0.9.2 (2020-05-26)
------------------

0.9.1 (2020-05-12)
------------------

0.9.0 (2020-04-30)
------------------
* avoid new deprecations (`#442 <https://github.com/ros2/demos/issues/442>`_)
* fix CMake warning about using uninitialized variables (`#439 <https://github.com/ros2/demos/issues/439>`_)
* Fix pendulum_control tests to use stdout stream. (`#430 <https://github.com/ros2/demos/issues/430>`_)
* code style only: wrap after open parenthesis if not in one line (`#429 <https://github.com/ros2/demos/issues/429>`_)
* Contributors: Chris Lalancette, Dirk Thomas, William Woodall

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
* Fixes to pendulum_control demo (`#385 <https://github.com/ros2/demos/issues/385>`_)
    * Add asserts to ensure that the latency is never negative
    * Switch last_sample to int64_t to match new rttest interface
    * Allow any number of spaces
    * Make sure to expect the extra newline for the pendulum_demo
    * Only publish statistics if they are available.
    * Remove some unused functions from rtt_executor.hpp
* Fix armhf build warnings (`#372 <https://github.com/ros2/demos/issues/372>`_)
* Contributors: Chris Lalancette, Prajakta Gokhale

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
* Removed unused lambda capture. (`#315 <https://github.com/ros2/demos/issues/315>`_)
* Added launch along with launch_testing as test dependencies. (`#313 <https://github.com/ros2/demos/issues/313>`_)
* Dropped legacy launch API usage. (`#311 <https://github.com/ros2/demos/issues/311>`_)
* Contributors: Emerson Knapp, Michel Hidalgo

0.6.2 (2019-01-15)
------------------

0.6.1 (2018-12-13)
------------------

0.6.0 (2018-12-07)
------------------
* Updated package maintainer. (`#286 <https://github.com/ros2/demos/issues/286>`_)
* Updated to match rmw_fastrtps_dynamic_cpp (`#271 <https://github.com/ros2/demos/issues/271>`_)
* Fixed spacing to comply with uncrusity 0.67 (`#267 <https://github.com/ros2/demos/issues/267>`_)
* Fixed no return code for main() in several files (`#266 <https://github.com/ros2/demos/issues/266>`_)
* Contributors: Dirk Thomas, Michael Carroll, Mikael Arguedas, testkit

0.5.1 (2018-06-28)
------------------
* make Mikael Arguedas the maintainer (`#263 <https://github.com/ros2/demos/issues/263>`_)
* Contributors: Mikael Arguedas

0.5.0 (2018-06-27)
------------------
* Updated launch files to account for the "old launch" getting renamespaced as ``launch`` -> ``launch.legacy``. (`#239 <https://github.com/ros2/demos/issues/239>`_)
* Fixed the pendulum's inertia in the physic model. (`#220 <https://github.com/ros2/demos/issues/220>`_)
* Contributors: Dirk Thomas, Thomas de Candia, William Woodall, dhood
