## **What Is This?**

This package provides several examples that demonstrate various inter-node communcations in ROS 2.

This package consists of the following examples:
1. `add_two_ints_client`
2. `listener_serialized_message`
3. `reuse_timer`
4. `add_two_ints_client_async`
5. `list_parameters`
6. `set_and_get_parameters`
7. `add_two_ints_server`
8. `list_parameters_async`
9. `set_and_get_parameters_async`
10. `allocator_tutorial`
11. `one_off_timer`
12. `set_parameters_callback`
13. `content_filtering_publisher`
14. `parameter_blackboard`
15. `talker`
16. `content_filtering_subscriber`
17. `parameter_event_handler`
18. `talker_loaned_message`
19. `even_parameters_node`
20. `parameter_events`
21. `talker_serialized_message`
22. `listener`
23. `parameter_events_async`
24. `listener_best_effort`
25. `matched_event_detect`

## **Build**

Run the command below to compile the `demo_nodes_cpp` ROS 2 package:

```bash
colcon build --packages-up-to demo_nodes_cpp
```

**Note**: By default, the demo executables will spin up the [SingleThreaded executor](https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Executors.html#executors) if run in separate processes, i.e., not composed in the same component container.
To configure the demo executables to run with a different executor, build the package with the custom `DEMO_EXECUTOR` flag set to the fully qualified name of the executor.
For example, to run with the experimental `EventsExecutor`,

```bash
colcon build --packages-select demo_nodes_cpp --cmake-args -DDEMO_EXECUTOR:STRING=rclcpp::experimental::executors::EventsExecutor
```

## **Run**

### Basic Talker & Listener

This runs a basic ROS 2 publisher and subscriber that exchanges the following string message with an incrementing integer:

> Hello World: <count_>

#### Talker

```bash
# Open new terminal
ros2 run demo_nodes_cpp talker
```

#### Listener [Default]

```bash
# Open new terminal
ros2 run demo_nodes_cpp listener
```

![](img/talker_listener.png)

#### Listener [Best Effort]

Compared to **Listener [Default]**, `listener_best_effort` runs a ROS 2 subscriber node that sets the Quality of Service (QoS) Reliability setting to **Best Effort** via the use of **rclcpp::SensorDataQoS**, as opposed to the default **Reliable**.

Messages sent using this policy configuration attempts to deliver samples but may lose them if the network is not robust.

```bash
# Open new terminal
ros2 run demo_nodes_cpp listener_best_effort
```

### Basic Server & Client

This runs a ROS 2 server that provides a service to process two integers, outputting the sum back to ROS 2 client node.

#### Server

```bash
# Open new terminal
ros2 run demo_nodes_cpp add_two_ints_server
```

#### Client [Synchronous]

```bash
# Open new terminal
ros2 run demo_nodes_cpp add_two_ints_client
```

#### Client [Asynchronous]

```bash
# Open new terminal
ros2 run demo_nodes_cpp add_two_ints_client_async
```

![](img/server_client.png)

### One-Off Timer

This runs `one_off_timer` that runs a periodic timer callback that **cancels** and **creates** a **Wall Timer** every **3 callbacks**.

```bash
ros2 run demo_nodes_cpp one_off_timer
```

![](img/one_off_timer.png)

### Reuse Timer

Similar to the previous demo, `reuse_timer` runs a periodic timer callback that **reuses the same Wall Timer** every **3 callbacks**.

```bash
ros2 run demo_nodes_cpp reuse_timer
```

### Serialized Messaging

This runs `talker_serialized_message` ROS 2 node that publishes a manual CDR serialization of the same string message.

```bash
# Open new terminal
ros2 run demo_nodes_cpp talker_serialized_message
```

This runs `listener_serialized_message` ROS 2 node that subscribes and prints out the serialized string message published by `talker_serialized_message`.
```bash
# Open new terminal
ros2 run demo_nodes_cpp listener_serialized_message
```

![](img/serialized_messaging.png)

### Content-Filter Messaging

This runs `content_filtering_subscriber` and `content_filtering_publisher` ROS 2 nodes which exchanges temperature data on the `/temperature` topic.

However the subscriber requests that data is only sent if the temperature is less than -30 C or greater than 100 C, saving bandwidth.

```bash
# Open new terminal
ros2 run demo_nodes_cpp content_filtering_subscriber
```

```bash
# Open new terminal
ros2 run demo_nodes_cpp content_filtering_publisher
```

![](img/content_filtering_messaging.png)

### List Parameters

This runs `list_parameters` ROS 2 node which simply programmatically list example parameter names and prefixes:

#### Synchronous

```bash
# Open new terminal
ros2 run demo_nodes_cpp list_parameters
```

#### Asynchronous

```bash
# Open new terminal
ros2 run demo_nodes_cpp list_parameters_async
```

### Set & Get Parameters

This runs `set_and_get_parameters` ROS 2 node which programmatically sets and gets parameters.

#### Synchronous

```bash
# Open new terminal
ros2 run demo_nodes_cpp set_and_get_parameters
```

#### Asynchronous

```bash
# Open new terminal
ros2 run demo_nodes_cpp set_and_get_parameters_async
```

### Allocator Tutorial

This runs `allocator_tutorial` ROS 2 node that publishes a `std_msgs/msg/UInt32` message that contains an integer representing the number of allocations and deallocations that happened during the program.

```bash
# Open new terminal
ros2 run demo_nodes_cpp allocator_tutorial
```
![](img/allocator_tutorial.png)

### Parameter Events

This runs `parameter_events`/`parameter_events_async` ROS 2 node(s) which initiates 10 parameter events which changes an example string parameter.

> foo -> bar -> baz -> foobar -> foo -> bar -> baz -> foobar -> foo -> bar

#### Synchronous

```bash
# Open new terminal
ros2 run demo_nodes_cpp parameter_events
```

#### Asynchronous

```bash
# Open new terminal
ros2 run demo_nodes_cpp parameters_events_async
```

### Even Parameters Node

This runs `even_parameters_node` ROS 2 node which shows a parameter callback that rejects all parameter updates except those that set an even integer.

```bash
# Open new terminal
ros2 run demo_nodes_cpp even_parameters_node
```

![](img/even_parameters_node.png)

### Set Parameters Callback

This runs `set_parameters_callback` ROS 2 node which triggers a callback when ROS 2 double parameter `param1` is set.

The callback then sets ROS 2 double parameter `param2` to a fixed `4.0`.

```bash
# Open new terminal
ros2 run demo_nodes_cpp set_parameters_callback
```

### Parameter Blackboard

This runs `parameter_blackboard` ROS 2 node which instantiates a ROS 2 parameter server which acts as a global "blackboard" for all nodes to get and set parameters.

```bash
# Open new terminal
ros2 run demo_nodes_cpp parameter_blackboard
```

![](img/parameters_blackboard.png)

### Parameter Event Handler

This runs `parameter_event_handler` ROS 2 node which monitors changes to the following parameters:

> node: "this_node"
>	parameter: "an_int_param"

>	node: "/a_namespace/a_remote_node"
>	parameter: "a_string_param"

```bash
# Open new terminal
ros2 run demo_nodes_cpp parameter_event_handler
```

### Loaned Messager Talker

This runs `loaned_message_talker` ROS 2 node that publishes unique messages which eliminates unnecessary copies throughout the ROS 2 stack to maximize performance.

```bash
# Open new terminal
ros2 run demo_nodes_cpp talker_loaned_message
```

### Matched Event Detect

This runs 3 ROS 2 nodes.
`matched_event_detect_node` node that set matched event callback for publisher and subscription separately to output connection or disconnection information.
`multi_sub_node` create/destroy subscriptions which connect the publisher of `matched_event_detect_node`.
`multi_pub_node` create/destroy publishers which connect the subscription of `matched_event_detect_node`.

```bash
# Open new terminal
ros2 run demo_nodes_cpp matched_event_detect
```

## **Verify**

### Basic Talker & Listener

When executed correctly, strings should be printed to terminal similar to what is shown below:

```bash
# In terminal running talker
[INFO] [1674551635.122315831] [talker]: Publishing: 'Hello World: 1'
[INFO] [1674551636.122275756] [talker]: Publishing: 'Hello World: 2'
[INFO] [1674551637.122274384] [talker]: Publishing: 'Hello World: 3'
[INFO] [1674551638.122235965] [talker]: Publishing: 'Hello World: 4'
[INFO] [1674551639.122277484] [talker]: Publishing: 'Hello World: 5'
#...
```

```bash
# In terminal running listener/listener_best_effort
[INFO] [1674551636.122881229] [listener]: I heard: [Hello World: 1]
[INFO] [1674551637.122832606] [listener]: I heard: [Hello World: 2]
[INFO] [1674551638.122675099] [listener]: I heard: [Hello World: 3]
[INFO] [1674551639.122788087] [listener]: I heard: [Hello World: 4]
[INFO] [1674551640.122850575] [listener]: I heard: [Hello World: 5]
#...
```

### Basic Server & Client

When executed correctly, strings should be printed to terminal similar to what is shown below:

#### Server
```bash
# In terminal running add_two_ints_server
[INFO] [1674553268.912391774] [add_two_ints_server]: Incoming request
a: 2 b: 3
```

#### Client [Synchronous]
```bash
# In terminal running add_two_ints_client
[INFO] [1674553268.912602310] [add_two_ints_client]: Result of add_two_ints: 5
```

#### Client [Asynchronous]
```bash
# In terminal running add_two_ints_client_async
[INFO] [1674553718.598690033] [add_two_ints_client]: Result of add_two_ints: 5
```

### One-Off Timer

When executed correctly, strings should be printed to terminal similar to what is shown below:

```bash
# In terminal running one_off_timer
[INFO] [1674552840.400680444] [one_off_timer]: in periodic_timer callback
[INFO] [1674552840.400915092] [one_off_timer]:   resetting one off timer
[INFO] [1674552841.401204072] [one_off_timer]: in one_off_timer callback
[INFO] [1674552842.400698264] [one_off_timer]: in periodic_timer callback
[INFO] [1674552842.400819311] [one_off_timer]:   not resetting one off timer
[INFO] [1674552844.400695640] [one_off_timer]: in periodic_timer callback
[INFO] [1674552844.400819933] [one_off_timer]:   not resetting one off timer
[INFO] [1674552846.400684373] [one_off_timer]: in periodic_timer callback
[INFO] [1674552846.400799868] [one_off_timer]:   resetting one off timer
[INFO] [1674552847.400948238] [one_off_timer]: in one_off_timer callback
[INFO] [1674552848.400703699] [one_off_timer]: in periodic_timer callback
[INFO] [1674552848.400828811] [one_off_timer]:   not resetting one off timer
[INFO] [1674552850.400680424] [one_off_timer]: in periodic_timer callback
[INFO] [1674552850.400775977] [one_off_timer]:   not resetting one off timer
[INFO] [1674552852.400690511] [one_off_timer]: in periodic_timer callback
[INFO] [1674552852.400815404] [one_off_timer]:   resetting one off timer
[INFO] [1674552853.401060139] [one_off_timer]: in one_off_timer callback
#...
```

> Notice how
> the timer is reset only after two callback iterations.

### Reuse Timer

When executed correctly, strings should be printed to terminal similar to what is shown below:

```bash
[INFO] [1674554333.056000613] [reuse_timer]: in periodic_timer callback
[INFO] [1674554333.056432269] [reuse_timer]:   resetting one off timer
[INFO] [1674554334.056732370] [reuse_timer]: in one_off_timer callback
[INFO] [1674554335.055978249] [reuse_timer]: in periodic_timer callback
[INFO] [1674554335.056111777] [reuse_timer]:   not resetting one off timer
[INFO] [1674554337.055978910] [reuse_timer]: in periodic_timer callback
[INFO] [1674554337.056108925] [reuse_timer]:   not resetting one off timer
[INFO] [1674554339.055966108] [reuse_timer]: in periodic_timer callback
[INFO] [1674554339.056083983] [reuse_timer]:   resetting one off timer
[INFO] [1674554340.056295399] [reuse_timer]: in one_off_timer callback
#...
```

> Same as One-Off Timer.

### Serialized Messaging

When executed correctly, strings should be printed to terminal similar to what is shown below:

```bash
# In terminal running talker_serialized_message
ROS message:
Hello World:1
serialized message:
00 01 00 00 0e 00 00 00 48 65 6c 6c 6f 20 57 6f 72 6c 64 3a 31 00
ROS message:
Hello World:2
serialized message:
00 01 00 00 0e 00 00 00 48 65 6c 6c 6f 20 57 6f 72 6c 64 3a 32 00
ROS message:
Hello World:3
serialized message:
00 01 00 00 0e 00 00 00 48 65 6c 6c 6f 20 57 6f 72 6c 64 3a 33 00
```
```bash
# In terminal running listerner_serialized_message
I heard data of length: 24
00 01 00 00 0e 00 00 00 48 65 6c 6c 6f 20 57 6f 72 6c 64 3a 32 00 00 00
serialized data after deserialization: Hello World:2
I heard data of length: 24
00 01 00 00 0e 00 00 00 48 65 6c 6c 6f 20 57 6f 72 6c 64 3a 33 00 00 00
serialized data after deserialization: Hello World:3
I heard data of length: 24
00 01 00 00 0e 00 00 00 48 65 6c 6c 6f 20 57 6f 72 6c 64 3a 34 00 00 00
serialized data after deserialization: Hello World:4
#...
```

### Content-Filtering Messaging

When executed correctly, strings should be printed to terminal similar to what is shown below:

```bash
# In terminal running content_filtering_publisher
[INFO] [1674563203.567530898] [content_filtering_publisher]: Publishing: '-100.000000'
[INFO] [1674563204.567508197] [content_filtering_publisher]: Publishing: '-90.000000'
[INFO] [1674563205.567517142] [content_filtering_publisher]: Publishing: '-80.000000'
#...
```

```bash
# In terminal running content_filtering_subscriber
[INFO] [1674563182.873825084] [content_filtering_subscriber]: subscribed to topic "/temperature" with content filter options "data < %0 OR data > %1, {-30.000000, 100.000000}"
[INFO] [1674563203.568310361] [content_filtering_subscriber]: I receive an emergency temperature data: [-100.000000]
[INFO] [1674563204.568128370] [content_filtering_subscriber]: I receive an emergency temperature data: [-90.000000]
[INFO] [1674563205.568082783] [content_filtering_subscriber]: I receive an emergency temperature data: [-80.000000]
#...
```

### List Parameters [Synchronous/Asynchronous]

When executed correctly, strings should be printed to terminal similar to what is shown below:

```bash
[INFO] [1674563905.346022942] [list_parameters]: Setting parameters...
[INFO] [1674563905.347158439] [list_parameters]: Listing parameters...
[INFO] [1674563905.347570888] [list_parameters]:
Parameter names:
 bar
 foo
 foo.first
 foo.second
Parameter prefixes:
 foo
```

### Set & Get Parameters [Synchronous/Asynchronous]

When executed correctly, strings should be printed to terminal similar to what is shown below:

```bash
[INFO] [1674564137.057616328] [set_and_get_parameters]:
Parameter name: foo
Parameter value (integer): 2
Parameter name: baz
Parameter value (double): 1.450000
Parameter name: foobarbaz
Parameter value (bool_array): [true, false]
Parameter name: toto
Parameter value (byte_array): [0xff, 0x7f]
```

### Allocator Tutorial

When executed correctly, strings should be printed to terminal similar to what is shown below:

```bash
This simple demo shows off a custom memory allocator to count all
instances of new/delete in the program.  It can be run in either regular
mode (no arguments), or in intra-process mode (by passing 'intra' as a
command-line argument).  It will then publish a message to the
'/allocator_tutorial' topic every 10 milliseconds until Ctrl-C is pressed.
At that time it will print a count of the number of allocations and
deallocations that happened during the program.

Intra-process pipeline is OFF.
```

Run `ros2 topic echo /allocator_tutorial` to see the output in the ROS 2 topic, `/allocator_tutorial`:
```bash
# Open new terminal
data: 224
---
data: 228
---
data: 230
---
data: 231
---
data: 233
---
data: 234
---
data: 235
---
```

### Parameter Events [Synchronous/Asynchronous]

When executed correctly, strings should be printed to terminal similar to what is shown below:

```bash
[INFO] [1674565202.370104660] [parameter_events]:
Parameter event:
 new parameters:
  foo
 changed parameters:
 deleted parameters:

[INFO] [1674565202.370241604] [parameter_events]:
Parameter event:
 new parameters:
  bar
 changed parameters:
 deleted parameters:

[INFO] [1674565202.370303487] [parameter_events]:
Parameter event:
 new parameters:
  baz
 changed parameters:
 deleted parameters:

[INFO] [1674565202.370355113] [parameter_events]:
Parameter event:
 new parameters:
  foobar
 changed parameters:
 deleted parameters:

[INFO] [1674565202.370398069] [parameter_events]:
Parameter event:
 new parameters:
 changed parameters:
  foo
 deleted parameters:

[INFO] [1674565202.370424143] [parameter_events]:
Parameter event:
 new parameters:
 changed parameters:
  bar
 deleted parameters:

[INFO] [1674565202.370447765] [parameter_events]:
Parameter event:
 new parameters:
 changed parameters:
  baz
 deleted parameters:

[INFO] [1674565202.370470405] [parameter_events]:
Parameter event:
 new parameters:
 changed parameters:
  foobar
 deleted parameters:

[INFO] [1674565202.370492871] [parameter_events]:
Parameter event:
 new parameters:
 changed parameters:
  foo
 deleted parameters:

[INFO] [1674565202.370515168] [parameter_events]:
Parameter event:
 new parameters:
 changed parameters:
  bar
 deleted parameters:
```

### Even Parameters Node

Run `ros2 param set /even_parameters_node myint 2` to set the parameter to a valid even integer and produce a similar result like below:

```bash
[INFO] [1674566014.892688389] [even_parameters_node]: parameter 'myint' has changed and is now: 2
```

Run `ros2 param set /even_parameters_node myint 3` to set the parameter to an invalid odd integer and produce a similar result like below:

```bash
[INFO] [1674566088.870030436] [even_parameters_node]: Requested value '3' for parameter 'myint' is not an even number: rejecting change...
```

### Set Parameters Callback

#### [Before]

Run `ros2 param get /set_param_callback_node param1` should print the following to terminal:
```bash
Double value is: 0.0
```

Run `ros2 param get /set_param_callback_node param2` should print the following to terminal:
```bash
Double value is 0.0
```

#### [Change]

Run `ros2 param set set_param_callback_node param1 10.0` and see it fail with
```bash
Setting parameter failed: cannot set 'param1' > 5.0
```

Run `ros2 param set set_param_callback_node param1 3.0`
```bash
Set parameter successful
```

#### [After]

Run `ros2 param get /set_param_callback_node param1` should print the following to terminal:
```bash
Double value is: 28.0
```

Run `ros2 param get /set_param_callback_node param2` should print the following to terminal:
```bash
Double value is 4.0
```

### Parameter Blackboard

When executed correctly, strings should be printed to terminal similar to what is shown below:

```bash
INFO] [1674568261.762813104] [parameter_blackboard]: Parameter blackboard node named '/parameter_blackboard' ready, and serving '5' parameters already!
```

Running `ros2 param list` should reveal the 6 parameters served:
```bash
/parameter_blackboard:
  qos_overrides./parameter_events.publisher.depth
  qos_overrides./parameter_events.publisher.durability
  qos_overrides./parameter_events.publisher.history
  qos_overrides./parameter_events.publisher.reliability
  start_type_description_service
  use_sim_time
```

### Parameter Event Handler

Run `ros2 param set this_node an_int_param 21` in a new terminal will produce the following results:
```bash
# In terminal running parameter_event_handler
[INFO] [1674569608.306038487] [this_node]: cb1: Received an update to parameter "an_int_param" of type integer: "21"
[INFO] [1674569608.306356753] [this_node]: cb3: Received an update to parameter "an_int_param" of type: integer: "21"
```

Run `ros2 param set /a_namespace/a_remote_node a_string_param "string value to set"` in a new terminal will produce the following results:
```bash
[INFO] [1674569622.728945232] [this_node]: cb2: Received an update to parameter "a_string_param" of type: string: "string value to set"
[INFO] [1674569622.729143396] [this_node]: cb3: Received an update to parameter "a_string_param" of type: string: "string value to set"
[INFO] [1674569622.729246614] [this_node]: cb3: Received an update to parameter "a_string_param" of type: string: "string value to set"
```

### Loaned Message Talker

When executed correctly, strings should be printed to terminal similar to what is shown below:

```bash
# In terminal running loaned_message_talker
[INFO] [1674570146.112222368] [loaned_message_talker]: Publishing: 'Hello World: 1'
[INFO] [1674570147.111670599] [loaned_message_talker]: Publishing: '2.000000'
[INFO] [1674570147.111853637] [loaned_message_talker]: Publishing: 'Hello World: 2'
[INFO] [1674570148.111662758] [loaned_message_talker]: Publishing: '3.000000'
[INFO] [1674570148.111804226] [loaned_message_talker]: Publishing: 'Hello World: 3'
[INFO] [1674570149.111651863] [loaned_message_talker]: Publishing: '4.000000'
[INFO] [1674570149.111819520] [loaned_message_talker]: Publishing: 'Hello World: 4'

```

> Note that Fast-DDS does not support the loaned messages. The loaned message API is used in iceoryx right now, which workes with CycloneDDS.

### Matched Event Detect

When executed correctly, strings should be printed to terminal similar to what is shown below:
```bash
#  In terminal running matched_event_detect
[INFO] [1679887690.127684740] [multi_sub_node]: Create a new subscription.
[INFO] [1679887690.128090105] [matched_event_detect_node]: First subscription is connected.
[INFO] [1679887690.128836774] [multi_sub_node]: Create a new subscription.
[INFO] [1679887690.129157780] [matched_event_detect_node]: The changed number of connected subscription is 1 and current number of connected subscription is 2.
[INFO] [1679887690.129193220] [multi_sub_node]: Destroy a subscription.
[INFO] [1679887690.130552475] [matched_event_detect_node]: The changed number of connected subscription is -1 and current number of connected subscription is 1.
[INFO] [1679887690.130588555] [multi_sub_node]: Destroy a subscription.
[INFO] [1679887690.131355128] [matched_event_detect_node]: Last subscription is disconnected.
[INFO] [1679887690.132014952] [multi_pub_node]: Create a new publisher.
[INFO] [1679887690.132262901] [matched_event_detect_node]: First publisher is connected.
[INFO] [1679887690.132898522] [multi_pub_node]: Create a new publisher.
[INFO] [1679887690.133143624] [matched_event_detect_node]: The changed number of connected publisher is 1 and current number of connected publisher is 2.
[INFO] [1679887690.133178687] [multi_pub_node]: Destroy a publisher.
[INFO] [1679887690.134139929] [matched_event_detect_node]: The changed number of connected publisher is -1 and current number of connected publisher is 1.
[INFO] [1679887690.134176647] [multi_pub_node]: Destroy a publisher.
[INFO] [1679887690.134887946] [matched_event_detect_node]: Last publisher is disconnected.
```

## **FAQ**

`Q`: Encountered the following error in terminal when running **Loaned Message Talker**:

```bash
[INFO] [1674570146.112148792] [rclcpp]: Currently used middleware can't loan messages. Local allocator will be used.
```

`A`: Ensure that **CycloneDDS** RMW is used by running `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`.

## **References**

1. [Zero-Copy via Loaned Messages](https://design.ros2.org/articles/zero_copy.html)
2. [ROS 2 Quality of Service Policies](https://design.ros2.org/articles/qos.html)
3. [Creating a content filtering subscription](https://docs.ros.org/en/rolling/Tutorials/Demos/Content-Filtering-Subscription.html)
