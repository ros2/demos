## **What Is This?**

This demo provides an example of further **Quality-Of-Service (QoS) configurability** as well as **Publication Modes** in a ROS 2 node via the use of `rmw_fastrtps_cpp` Application Programming Interface (API).

The example utilitises `eprosima::fastdds::statistics::dds::DomainParticipant` as well as `eprosima::fastdds::dds::DataWriter`.

A `DomainParticipant` defines a singular working unit which groups a set of **Publishers** and **Subscribers** together. See **References - DomainParticipant** for more details.

A `DataWriter` is created by a **Publisher** and writes data to a topic with configurable behaviors. See **References - DataWriter** for more details.

## **Build**

```bash
colcon build --packages-up-to demo_nodes_cpp_native
```

## **Run**

```bash
ros2 run demo_nodes_cpp_native talker
```

## **Verify**

When executed correctly, the following strings should be printed to the terminal similar to what is shown below:

```bash
[INFO] [1674525877.083735645] [talker_native]: eprosima::fastdds::dds::DomainParticipant * 94193367466752
[INFO] [1674525877.084105822] [talker_native]: eprosima::fastdds::dds::DataWriter * 94193370040688
[INFO] [1674525877.584006930] [talker_native]: Publishing: 'Hello World: 1'
[INFO] [1674525878.083967966] [talker_native]: Publishing: 'Hello World: 2'
[INFO] [1674525878.583917242] [talker_native]: Publishing: 'Hello World: 3'
[INFO] [1674525879.083963276] [talker_native]: Publishing: 'Hello World: 4'
[INFO] [1674525879.583918839] [talker_native]: Publishing: 'Hello World: 5'
#...
```

## **FAQ**

`Q`: Why use `rmw_fastrtps_cpp` API directly?

`A`: ROS 2 only allows for the configuration of certain middleware QoS (see [ROS 2 QoS policies](https://docs.ros.org/en/foxy/Concepts/About-Quality-of-Service-Settings.html#qos-policies)). In addition to ROS 2 QoS policies, `rmw_fastrtps` sets two more **Fast DDS** configurable parameters:

>**History memory policy**: `PREALLOCATED_WITH_REALLOC_MEMORY_MODE`

>**Publication mode**: `ASYNCHRONOUS_PUBLISH_MODE`

In addition, `rmw_fastrtps` offers the possibility to further configure **Fast DDS**:

>[Change publication mode](https://github.com/ros2/rmw_fastrtps#change-publication-mode)

>[Full QoS configuration](https://github.com/ros2/rmw_fastrtps#full-qos-configuration)

`Q`: Is it still called `Fast RTPS`?

`A`: No. `Fast RTPS` has been renamed to `Fast DDS` since `ROS 2 Foxy`.

## **References**

1. [Fast DDS](https://www.eprosima.com/index.php/products-all/eprosima-fast-dds)
2. [What Is RTPS?](https://www.eprosima.com/index.php/resources-all/whitepapers/rtps)
3. [Fast RTPS Becomes Fast DDS in Foxy](https://discourse.ros.org/t/fast-rtps-becomes-fast-dds-in-foxy/15020/2)
4. [Advanced Usage](https://github.com/ros2/rmw_fastrtps#advance-usage)
5. [DomainParticipant](https://fast-dds.docs.eprosima.com/en/latest/fastdds/api_reference/dds_pim/domain/domainparticipant.html)
6. [DataWriter](https://fast-dds.docs.eprosima.com/en/latest/fastdds/api_reference/dds_pim/publisher/datawriter.html#datawriter)
