^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package demo_nodes_cpp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.33.5 (2024-09-06)
-------------------

0.33.4 (2024-06-27)
-------------------

0.33.3 (2024-05-13)
-------------------
* [demo_nodes_cpp] some readme and executable name fixups (`#678 <https://github.com/ros2/demos/issues/678>`_) (`#688 <https://github.com/ros2/demos/issues/688>`_)
  (cherry picked from commit aa8df8904b864d063e31fd5b953ffe561c7a9fe0)
  Co-authored-by: Mikael Arguedas <mikael.arguedas@gmail.com>
* Fix gcc warnings when building with optimizations. (`#672 <https://github.com/ros2/demos/issues/672>`_) (`#673 <https://github.com/ros2/demos/issues/673>`_)
  * Fix gcc warnings when building with optimizations.
  When building the allocator_tutorial_pmr demo with -O2,
  gcc is throwing an error saying that new and delete are
  mismatched.  This is something of a misnomer, however;
  the real problem is that the global new override we
  have in that demo is actually implemented incorrectly.
  In particular, the documentation at
  https://en.cppreference.com/w/cpp/memory/new/operator_new
  very clearly specifies that operator new either has to
  return a valid pointer, or throw an exception on error.
  Our version wasn't throwing the exception, so change it
  to throw std::bad_alloc if std::malloc fails.
  While we are in here, also fix another small possible
  is where std::malloc could return nullptr on a zero-sized
  object, thus throwing an exception it shouldn't.
  * Always inline the new and delete operators.
  That's because gcc 13 has a bug where it can sometimes
  inline one or the other, and then it detects that they
  mismatch.  For gcc and clang, just force them to always
  be inline in this demo.
  * Switch to NOINLINE instead.
  Both clang and MSVC don't like inlining these, so instead
  ensure that they are *not* inlined.  This also works
  because the problem is when new is inlined but not delete
  (or vice-versa).  As long as they are both not inlined,
  this should fix the warning.
  (cherry picked from commit 957ddbb9f04f55cabd8496e8d74eb35ee4d29105)
  Co-authored-by: Chris Lalancette <clalancette@gmail.com>
* Contributors: mergify[bot]

0.33.2 (2024-03-28)
-------------------
* A few uncrustify fixes for 0.78. (`#667 <https://github.com/ros2/demos/issues/667>`_)
* Allow users to configure the executor for executables in `demo_nodes_cpp` (`#666 <https://github.com/ros2/demos/issues/666>`_)
* Update maintainer list in package.xml files (`#665 <https://github.com/ros2/demos/issues/665>`_)
* Contributors: Chris Lalancette, Michael Jeronimo, Yadu

0.33.1 (2024-02-07)
-------------------

0.33.0 (2024-01-24)
-------------------

0.32.1 (2023-12-26)
-------------------
* Added extra documentation and clarifications. (`#651 <https://github.com/ros2/demos/issues/651>`_)
* Contributors: jrutgeer

0.32.0 (2023-11-06)
-------------------
* Add in support for both the PMR and custom allocator tutorials. (`#655 <https://github.com/ros2/demos/issues/655>`_)
* Replacing old-style C++ allocator with a polymorphic memory resource (PMR) (`#653 <https://github.com/ros2/demos/issues/653>`_)
* Contributors: Ali Ashkani Nia, Chris Lalancette

0.31.1 (2023-09-07)
-------------------
* Remove unnecessary captures in the various demos. (`#647 <https://github.com/ros2/demos/issues/647>`_)
* Contributors: Chris Lalancette

0.31.0 (2023-08-21)
-------------------
* Dramatically speed up the demo_nodes_cpp tests (`#641 <https://github.com/ros2/demos/issues/641>`_)
* Switch to using RCLCPP logging macros in the lifecycle package. (`#644 <https://github.com/ros2/demos/issues/644>`_)
* Contributors: Chris Lalancette

0.30.1 (2023-07-11)
-------------------
* failed to call introspection_client (`#643 <https://github.com/ros2/demos/issues/643>`_)
* Contributors: Chen Lihui

0.30.0 (2023-06-12)
-------------------
* Small cleanups to the demos when running through them. (`#639 <https://github.com/ros2/demos/issues/639>`_)
* Cleanup demo_nodes_cpp CMake and dependencies (`#638 <https://github.com/ros2/demos/issues/638>`_)
* Change the service introspection parameter off value to 'disabled' (`#634 <https://github.com/ros2/demos/issues/634>`_)
* Contributors: Chris Lalancette

0.29.0 (2023-06-07)
-------------------
* Add demos for using logger service (`#611 <https://github.com/ros2/demos/issues/611>`_)
* Contributors: Barry Xu

0.28.1 (2023-05-11)
-------------------

0.28.0 (2023-04-27)
-------------------

0.27.0 (2023-04-13)
-------------------
* Change all ROS2 -> ROS 2. (`#610 <https://github.com/ros2/demos/issues/610>`_)
* Add matched event demo for rclcpp and rclpy (`#607 <https://github.com/ros2/demos/issues/607>`_)
* Contributors: Barry Xu, Chris Lalancette

0.26.0 (2023-04-11)
-------------------
* Fix the set_parameters_callback example program. (`#608 <https://github.com/ros2/demos/issues/608>`_)
* [demo_nodes_cpp] Add YAML launch demos for topics (`#605 <https://github.com/ros2/demos/issues/605>`_)
* update launch file name format to match documentation (`#588 <https://github.com/ros2/demos/issues/588>`_)
* Contributors: Chris Lalancette, Damien LaRocque, Patrick Wspanialy

0.25.0 (2023-03-01)
-------------------
* Service introspection (`#602 <https://github.com/ros2/demos/issues/602>`_)
  * Add in a rclcpp and rclpy demo of introspection.
* Contributors: Chris Lalancette

0.24.1 (2023-02-24)
-------------------
* Added README.md for demo_cpp_nodes (`#599 <https://github.com/ros2/demos/issues/599>`_)
* Contributors: Gary Bey

0.24.0 (2023-02-14)
-------------------
* Update the demos to C++17. (`#594 <https://github.com/ros2/demos/issues/594>`_)
* [rolling] Update maintainers - 2022-11-07 (`#589 <https://github.com/ros2/demos/issues/589>`_)
* Contributors: Audrow Nash, Chris Lalancette

0.23.0 (2022-11-02)
-------------------
* Demo for pre and post set parameter callback support (`#565 <https://github.com/ros2/demos/issues/565>`_)
  * local parameter callback support
* Contributors: Deepanshu Bansal

0.22.0 (2022-09-13)
-------------------
* counter starts from 1, not 2. (`#562 <https://github.com/ros2/demos/issues/562>`_)
* add a demo of content filter listener (`#557 <https://github.com/ros2/demos/issues/557>`_)
* Contributors: Chen Lihui, Tomoya Fujita

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
* Add how to fix the most vexing parse problem (`#541 <https://github.com/ros2/demos/issues/541>`_)
  * use uniform initialization
* Contributors: Audrow Nash, Tomoya Fujita

0.17.0 (2021-10-18)
-------------------
* Fixing deprecated subscriber callback warnings (`#532 <https://github.com/ros2/demos/issues/532>`_)
* Contributors: Abrar Rahman Protyasha

0.16.0 (2021-08-11)
-------------------
* Update talker_loaned_message.cpp (`#518 <https://github.com/ros2/demos/issues/518>`_)
* Contributors: Zongbao Feng

0.15.0 (2021-05-14)
-------------------
* Revert "Use sizeof(char) in place for sizeof(void) (`#515 <https://github.com/ros2/demos/issues/515>`_)" (`#516 <https://github.com/ros2/demos/issues/516>`_)
* change how serialized message works with subscription (`#497 <https://github.com/ros2/demos/issues/497>`_)
* Use sizeof(char) in place for sizeof(void) (`#515 <https://github.com/ros2/demos/issues/515>`_)
* Fix small print issue in allocator tutorial. (`#509 <https://github.com/ros2/demos/issues/509>`_)
* Contributors: Chris Lalancette, Michel Hidalgo, William Woodall

0.14.2 (2021-04-26)
-------------------
* Small fixes for even_parameters_node. (`#500 <https://github.com/ros2/demos/issues/500>`_)
* Contributors: Chris Lalancette

0.14.1 (2021-04-19)
-------------------

0.14.0 (2021-04-06)
-------------------
* change ParameterEventHandler to take events as const ref instead of shared pointer (`#494 <https://github.com/ros2/demos/issues/494>`_)
* Fix integer type in RCLCPP\_* macro printf. (`#492 <https://github.com/ros2/demos/issues/492>`_)
* Contributors: Chris Lalancette, William Woodall

0.13.0 (2021-03-25)
-------------------
* Add a demo for the new ParameterEventHandler class (`#486 <https://github.com/ros2/demos/issues/486>`_)
* Contributors: Michael Jeronimo

0.12.1 (2021-03-18)
-------------------
* Filter qos overrides in paramter events demos (`#491 <https://github.com/ros2/demos/issues/491>`_)
* Update code now that parameter types are static by default (`#487 <https://github.com/ros2/demos/issues/487>`_)
* Contributors: Ivan Santiago Paunovic

0.12.0 (2021-01-25)
-------------------
* Update logging macros (`#476 <https://github.com/ros2/demos/issues/476>`_)
* Contributors: Audrow Nash

0.11.0 (2020-12-10)
-------------------
* Make sure to wait for the service before declaring events. (`#473 <https://github.com/ros2/demos/issues/473>`_)
* Update the package.xml files with the latest Open Robotics maintainers (`#466 <https://github.com/ros2/demos/issues/466>`_)
* Contributors: Chris Lalancette, Michael Jeronimo

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
* avoid new deprecations (`#442 <https://github.com/ros2/demos/issues/442>`_)
* use serialized message (`#441 <https://github.com/ros2/demos/issues/441>`_)
* Replace deprecated launch_ros usage (`#437 <https://github.com/ros2/demos/issues/437>`_)
* code style only: wrap after open parenthesis if not in one line (`#429 <https://github.com/ros2/demos/issues/429>`_)
* Use `spin_until_future_complete` instead of `spin_some` in parameters_event demo (`#427 <https://github.com/ros2/demos/issues/427>`_)
* change the logging demo test for updated console format (`#421 <https://github.com/ros2/demos/issues/421>`_)
* [demo_nodes_cpp]  Add XML launch demos (`#419 <https://github.com/ros2/demos/issues/419>`_)
* Contributors: Dirk Thomas, Ivan Santiago Paunovic, Jacob Perron, Karsten Knese, Steven Macenski, William Woodall, Yutaka Kondo

0.8.4 (2019-11-19)
------------------
* Add in a more helpful usage message to allocator_tutorial. (`#409 <https://github.com/ros2/demos/issues/409>`_)
* Contributors: Chris Lalancette

0.8.3 (2019-11-11)
------------------

0.8.2 (2019-11-08)
------------------
* Don't redefine add_dependencies (`#408 <https://github.com/ros2/demos/issues/408>`_)
* Contributors: Dan Rose

0.8.1 (2019-10-23)
------------------
* rename return functions for loaned messages (`#403 <https://github.com/ros2/demos/issues/403>`_)
* Replace ready_fn with ReadyToTest action (`#404 <https://github.com/ros2/demos/issues/404>`_)
* remove intra-process manager impl (`#382 <https://github.com/ros2/demos/issues/382>`_)
* zero copy api (`#394 <https://github.com/ros2/demos/issues/394>`_)
* Remove command line parsing from C++ demos (`#401 <https://github.com/ros2/demos/issues/401>`_)
* Need to specify NodeOption explicitly to allow declaration. (`#389 <https://github.com/ros2/demos/issues/389>`_)
* Contributors: Alberto Soragna, Jacob Perron, Karsten Knese, Peter Baughman, tomoya

0.8.0 (2019-09-26)
------------------
* Adding visibility macros to demos (`#381 <https://github.com/ros2/demos/issues/381>`_)
* Demos using composition (`#375 <https://github.com/ros2/demos/issues/375>`_)
* Contributors: Siddharth Kucheria

0.7.6 (2019-05-30)
------------------

0.7.5 (2019-05-29)
------------------
* Update to use new parameter option names (`#355 <https://github.com/ros2/demos/issues/355>`_)
* Contributors: William Woodall

0.7.4 (2019-05-20)
------------------

0.7.3 (2019-05-10)
------------------
* Added the ``parameter_blackboard`` demo to ``demo_nodes_cpp`` to make some tutorials easier. (`#333 <https://github.com/ros2/demos/issues/333>`_)
* Contributors: William Woodall

0.7.2 (2019-05-08)
------------------
* changes to avoid deprecated API's (`#332 <https://github.com/ros2/demos/issues/332>`_)
* Corrected publish calls with shared_ptr signature (`#327 <https://github.com/ros2/demos/issues/327>`_)
* Migrate launch tests to new launch_testing features & API (`#318 <https://github.com/ros2/demos/issues/318>`_)
* Contributors: Michel Hidalgo, William Woodall, ivanpauno

0.7.1 (2019-04-26)
------------------
* Updated to declare parameters. (`#241 <https://github.com/ros2/demos/issues/241>`_)
* Contributors: Shane Loretz

0.7.0 (2019-04-14)
------------------
* Moved away from deprecated rclcpp APIs. (`#321 <https://github.com/ros2/demos/issues/321>`_)
* Added launch along with launch_testing as test dependencies. (`#313 <https://github.com/ros2/demos/issues/313>`_)
* Updated for NodeOptions Node constructor. (`#308 <https://github.com/ros2/demos/issues/308>`_)
* Contributors: Emerson Knapp, Michael Carroll, Michel Hidalgo

0.6.2 (2019-01-15)
------------------

0.6.1 (2018-12-13)
------------------

0.6.0 (2018-12-07)
------------------
* Added semicolons to all RCLCPP and RCUTILS macros. (`#278 <https://github.com/ros2/demos/issues/278>`_)
* Removed parameter node, all nodes take parameter by default now (`#265 <https://github.com/ros2/demos/issues/265>`_)
* Added example of registering custom parameter validation callbacks (`#273 <https://github.com/ros2/demos/issues/273>`_)
* Removed imu_listener node (`#272 <https://github.com/ros2/demos/issues/272>`_)
* Refined demo_nodes_cpp source codes (`#269 <https://github.com/ros2/demos/issues/269>`_)
* Fixed typo in comment (`#268 <https://github.com/ros2/demos/issues/268>`_)
* Removed rosidl deps as this package doesnt generate any messages (`#264 <https://github.com/ros2/demos/issues/264>`_)
* Fixed no return code for main() in several files (`#266 <https://github.com/ros2/demos/issues/266>`_)
* Contributors: Chris Lalancette, Mikael Arguedas, Yutaka Kondo, testkit

0.5.1 (2018-06-28)
------------------

0.5.0 (2018-06-27)
------------------
* Reduced the publishing of the allocator_tutorial to 100Hz. (`#257 <https://github.com/ros2/demos/issues/257>`_)
  * Signed-off-by: Chris Lalancette <clalancette@openrobotics.org>
* Removed the now obsolete ros2param executable, use ``ros2 param`` instead. (`#251 <https://github.com/ros2/demos/issues/251>`_)
* Fixed a potiential nullptr dereference issue in ``demo_nodes_cpp``. (`#242 <https://github.com/ros2/demos/issues/242>`_)
* Added demo nodes which use the new serialized message typed publishers and subscriptions. (`#185 <https://github.com/ros2/demos/issues/185>`_)
* Added a new-style launch file for the talker and listener demo nodes, called ``talker_listener.launch.py``. (`#244 <https://github.com/ros2/demos/issues/244>`_)
* Updated launch files to account for the "old launch" getting renamespaced as ``launch`` -> ``launch.legacy``. (`#239 <https://github.com/ros2/demos/issues/239>`_)
* Updated to handle refactor of the ``ParameterVariant`` class. (`#237 <https://github.com/ros2/demos/issues/237>`_)
* Updated to account for the fact that the ROS Parameter services starts automatically now. (`#236 <https://github.com/ros2/demos/issues/236>`_)
* Added some uses of parameter arrays to the ``set_and_get_parameters`` demo. (`#235 <https://github.com/ros2/demos/issues/235>`_)
* Contributors: Chris Lalancette, Dirk Thomas, Karsten Knese, Mikael Arguedas, Shane Loretz, William Woodall, cshen
