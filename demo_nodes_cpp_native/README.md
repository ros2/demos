## **What Is This?**

This demo provides an example of creating and running a ROS 2 node using the native **rmw_fastrtps_cpp** API directly, rather than doing so through vanilla **rclcpp**.

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

`Q`: Why use native `rmw_fastrtps_cpp` API directly, instead of using `rclcpp`?

`A`: Doing so would allow for better fault isolation and debugging. However, it would be with the obvious drawback of having to include `rmw_fastrtps_cpp` dependency manually.

`Q`: Is it still called `Fast RTPS`?

`A`: No. `Fast RTPS` has been renamed to `Fast DDS` since `ROS 2 Foxy`.

## **References**

1. [Fast DDS](https://www.eprosima.com/index.php/products-all/eprosima-fast-dds)
2. [What Is RTPS?](https://www.eprosima.com/index.php/resources-all/whitepapers/rtps)
3. [Fast RTPS Becomes Fast DDS in Foxy](https://discourse.ros.org/t/fast-rtps-becomes-fast-dds-in-foxy/15020/2)
