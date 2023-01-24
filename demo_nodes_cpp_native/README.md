## **What Is This?**

This demo ... 

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

WIP

## **References**

1. 
