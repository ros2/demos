^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package lifecycle
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.33.5 (2024-09-06)
-------------------

0.33.4 (2024-06-27)
-------------------

0.33.3 (2024-05-13)
-------------------

0.33.2 (2024-03-28)
-------------------
* A few uncrustify fixes for 0.78. (`#667 <https://github.com/ros2/demos/issues/667>`_)
* Update maintainer list in package.xml files (`#665 <https://github.com/ros2/demos/issues/665>`_)
* Contributors: Chris Lalancette, Michael Jeronimo

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
* Switch to using RCLCPP logging macros in the lifecycle package. (`#644 <https://github.com/ros2/demos/issues/644>`_)
* Contributors: Chris Lalancette

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
* update launch file name format to match documentation (`#588 <https://github.com/ros2/demos/issues/588>`_)
* Contributors: Patrick Wspanialy

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
* Make lifecycle demo automatically exit when done (`#558 <https://github.com/ros2/demos/issues/558>`_)
* Contributors: Shane Loretz

0.20.0 (2022-03-01)
-------------------
* Use default on_activate()/on_deactivate() implemenetation of Node (`#552 <https://github.com/ros2/demos/issues/552>`_)
* Contributors: Ivan Santiago Paunovic

0.19.0 (2022-01-14)
-------------------

0.18.0 (2021-12-17)
-------------------
* Update maintainers to Audrow Nash and Michael Jeronimo (`#543 <https://github.com/ros2/demos/issues/543>`_)
* Contributors: Audrow Nash

0.17.0 (2021-10-18)
-------------------
* Fix use of future in lifecycle demo (`#534 <https://github.com/ros2/demos/issues/534>`_)
* Fixing deprecated subscriber callback warnings (`#532 <https://github.com/ros2/demos/issues/532>`_)
* Contributors: Abrar Rahman Protyasha, Christophe Bedard

0.16.0 (2021-08-11)
-------------------

0.15.0 (2021-05-14)
-------------------

0.14.2 (2021-04-26)
-------------------
* Cleanup the README.rst for the lifecycle demo. (`#508 <https://github.com/ros2/demos/issues/508>`_)
* Contributors: Chris Lalancette

0.14.1 (2021-04-19)
-------------------

0.14.0 (2021-04-06)
-------------------
* change ParameterEventHandler to take events as const ref instead of shared pointer (`#494 <https://github.com/ros2/demos/issues/494>`_)
* Contributors: William Woodall

0.13.0 (2021-03-25)
-------------------

0.12.1 (2021-03-18)
-------------------

0.12.0 (2021-01-25)
-------------------

0.11.0 (2020-12-10)
-------------------
* Update the package.xml files with the latest Open Robotics maintainers (`#466 <https://github.com/ros2/demos/issues/466>`_)
* Contributors: Michael Jeronimo

0.10.1 (2020-09-21)
-------------------
* Add missing required parameter in LifecycleNode launch action (`#456 <https://github.com/ros2/demos/issues/456>`_)
* Contributors: Ivan Santiago Paunovic

0.10.0 (2020-06-17)
-------------------

0.9.3 (2020-06-01)
------------------

0.9.2 (2020-05-26)
------------------
* Fix typo (`#445 <https://github.com/ros2/demos/issues/445>`_)
* Replace ``ros2 msg`` command in lifecycle README (`#446 <https://github.com/ros2/demos/issues/446>`_)
* Contributors: Audrow Nash, Shota Aoki

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
* Remove unnecessary dependency on ros2run (`#413 <https://github.com/ros2/demos/issues/413>`_)
* Contributors: Michel Hidalgo

0.8.1 (2019-10-23)
------------------
* Replace ready_fn with ReadyToTest action (`#404 <https://github.com/ros2/demos/issues/404>`_)
* Contributors: Peter Baughman

0.8.0 (2019-09-26)
------------------
* Fix lifecycle_service_client namespace (`#369 <https://github.com/ros2/demos/issues/369>`_)
* Contributors: Cameron Evans

0.7.6 (2019-05-30)
------------------

0.7.5 (2019-05-29)
------------------
* Update asciinema recordings (`#360 <https://github.com/ros2/demos/issues/360>`_)
* Use rate instead of thread::sleep to react to Ctrl-C (`#348 <https://github.com/ros2/demos/issues/348>`_)
* Contributors: Dirk Thomas, Karsten Knese

0.7.4 (2019-05-20)
------------------
* Add lifecycle rostest (`#336 <https://github.com/ros2/demos/issues/336>`_)
* Contributors: Michel Hidalgo

0.7.3 (2019-05-10)
------------------

0.7.2 (2019-05-08)
------------------
* changes to avoid deprecated API's (`#332 <https://github.com/ros2/demos/issues/332>`_)
* Corrected publish calls with shared_ptr signature (`#327 <https://github.com/ros2/demos/issues/327>`_)
* Contributors: William Woodall, ivanpauno

0.7.1 (2019-04-26)
------------------

0.7.0 (2019-04-14)
------------------
* Updated for NodeOptions Node constructor. (`#308 <https://github.com/ros2/demos/issues/308>`_)
* Contributors: Michael Carroll

0.6.2 (2019-01-15)
------------------
* Added readme.rst (`#300 <https://github.com/ros2/demos/issues/300>`_)
* Contributors: Karsten Knese

0.6.1 (2018-12-13)
------------------

0.6.0 (2018-12-07)
------------------
* Cleaned up lifecycle demo (`#283 <https://github.com/ros2/demos/issues/283>`_)
* Updated for refactoring in rclcpp (`#276 <https://github.com/ros2/demos/issues/276>`_)
* Added semicolons to all RCLCPP and RCUTILS macros. (`#278 <https://github.com/ros2/demos/issues/278>`_)
* Fixed typo in comment (`#270 <https://github.com/ros2/demos/issues/270>`_)
* Contributors: Chris Lalancette, Karsten Knese, Yutaka Kondo

0.5.1 (2018-06-28)
------------------

0.5.0 (2018-06-27)
------------------
* Converted launch files to the new launch style. (`#262 <https://github.com/ros2/demos/issues/262>`_)
* Updated to support remapping arguments to python nodes by passing unused arguments to rclpy from argparse. (`#252 <https://github.com/ros2/demos/issues/252>`_)
* Updated to handle change in signature to ``get_service_name``. (`#245 <https://github.com/ros2/demos/issues/245>`_)
* Updated launch files to account for the "old launch" getting renamespaced as ``launch`` -> ``launch.legacy``. (`#239 <https://github.com/ros2/demos/issues/239>`_)
* Updated service client demos to handle multiple requests. (`#228 <https://github.com/ros2/demos/issues/228>`_)
* Contributors: Geoffrey Biggs, Kevin Allen, Shane Loretz, William Woodall, dhood
