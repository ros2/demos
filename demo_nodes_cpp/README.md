## **What Is This?**

This demo ...

## **Build**

Run the command below to compile the `demo_nodes_cpp` ROS 2 package:

```bash
colcon build --packages-up-to demo_nodes_cpp
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

Compared to **Listener [Default]**, `listener_best_effort` runs a ROS2 subscribe node that sets the Quality of Service (QoS) Reliability setting to **Best Effort**, as opposed to the default **Reliable**. Messages sent using this policy configuration attempts to deliver samples but may lose them if the network is not robust.

```bash
# Open new terminal
ros2 run demo_nodes_cpp listener_best_effort
```

### Basic Server & Client

This runs a ROS 2 server that provides the service to processes two integers and outputs its sum back to ROS 2 client node. 

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

Similar to previous, this runs `reuse_timer` that runs a periodic timer callback that **reuses the same Wall Timer** every **3 callbacks**.

```bash
ros2 run demo_nodes_cpp reuse_timer
```

### Serialized Messaging

This runs `talker_serialized_message` ROS 2 node that publishes a manual CDR serialization of the same string message.

```bash
# Open new terminal
ros2 run demo_nodes_cpp talker_serialized_message
```

This runs `listener_serialized_message` ROS2 2 node that subscribes and prints out the serialized string message published by `talker_serialized_message`. 
```bash
# Open new terminal
ros2 run demo_nodes_cpp listener_serialized_message
```

![](img/serialized_messaging.png)


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

> Notice how the timer is reset only after two callback iterations.

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
```
## **FAQ**

`Q`: 

`A`: 

## **References**

1. [Zero-Copy via Loaned Messages](https://design.ros2.org/articles/zero_copy.html)
2. [ROS2 Quality of Service Policies](https://design.ros2.org/articles/qos.html)
3. []()
