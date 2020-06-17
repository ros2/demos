^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pendulum_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
