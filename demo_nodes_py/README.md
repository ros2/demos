## **What Is This?**

This package contains the following ROS 2 nodes:

1. `listener`
2. `talker`
3. `add_two_ints_client`
4. `add_two_ints_server`
5. `set_parameters_callback`
6. `add_two_ints_client_async`
7. `listener_qos`
8. `talker_qos`
9. `listerner_serialized`
10. `async_param_client`

## **Build**

Run this command to build the package:

```bash
colcon build --packages-select demo_nodes_py
```

## **Run**

### Basic Listener & Talker

This runs `talker` and `listener` ROS 2 nodes which exchange the following string with incremeting integer:

> Hello World: <count_>

```bash
# Open new terminal
ros2 run demo_nodes_py talker
```

```bash
# Open new terminal
ros2 run demo_nodes_py listener
```

![](img/talker_listener.png)

### Server & Client

This runs `add_two_ints_client` and `add_two_ints_server` ROS 2 client and server where the server processes two integers sent from the client and publishes its sum to the client to be printed out.

#### Server

```bash
# Open new terminal
ros2 run demo_nodes_py add_two_ints_server
```

#### Client [Synchronous]

```bash
# Open new terminal
ros2 run demo_nodes_py add_two_ints_client
```

#### Client [Asynchronous]

```bash
# Open new terminal
ros2 run demo_nodes_py add_two_ints_client_async
```
![](img/server_client.png)

### QoS Listener & Talker

Similar to previous, this runs `talker_qos` and `listener_qos` ROS 2 nodes which exchange messages using the Quality of Service (QoS) best effort policy.

```bash
# Open new terminal
ros2 run demo_nodes_py talker_qos
```

```bash
# Open new terminal
ros2 run demo_nodes_py listener_qos
```

![](img/qos_listener_talker.png)


### Set Parameters Callback

This runs `set_parameters_callback` ROS 2 node which triggers a callback when ROS 2 double parameter `param1` is set. The callback then sets ROS 2 double parameter `param2` to a fixed `4.0`:

```bash
# Open new terminal
ros2 run demo_nodes_py set_parameters_callback
```

![](img/set_parameters_callback.png)

### Asynchronous Parameter Client

This runs `async_param_client` ROS 2 node which demonstrates how to set, list, get, load and delete parameters from the parameter blackboard in an asynchronous manner:

```bash
# Open new terminal
ros2 run demo_nodes_cpp parameter_blackboard
```

> For more details on parameter_blackboard, please refer to demo_nodes_cpp README.md.

```bash
# Open new terminal
ros2 run demo_nodes_py async_param_client
```

### Serialized Subscriber

This runs `serialized_subscriber` ROS 2 node that subscribes to the ROS 2 topic, `/chatter`, and deserializes as well as prints out received strings:

```bash
# Open new terminal
ros2 run demo_nodes_py listener_serialized
```

![](img/serialized_subscriber.png)

## **Verify**

### Basic Listener & Talker

When executed correctly, strings should be printed to the terminal similar to what is shown below:

```bash
# In terminal running talker
[INFO] [1674573022.635075580] [talker]: Publishing: "Hello World: 0"
[INFO] [1674573023.593075728] [talker]: Publishing: "Hello World: 1"
[INFO] [1674573024.592438479] [talker]: Publishing: "Hello World: 2"
```

```bash
# In terminal running listener
[INFO] [1674573025.634810166] [listener]: I heard: [Hello World: 0]
[INFO] [1674573026.596392653] [listener]: I heard: [Hello World: 1]
[INFO] [1674573027.596400384] [listener]: I heard: [Hello World: 2]
#...
```

### Basic Server & Client

When executed correctly, strings should be printed to the terminal similar to what is shown below:

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

### QoS Listener & Talker

When executed correctly, strings should be printed to the terminal similar to what is shown below:

```bash
# In terminal running talker_qos
[INFO] [1674572630.018298164] [talker_qos]: Best effort talker
[INFO] [1674572631.021168516] [talker_qos]: Publishing: "Hello World: 0"
[INFO] [1674572632.021072723] [talker_qos]: Publishing: "Hello World: 1"
[INFO] [1674572633.021197421] [talker_qos]: Publishing: "Hello World: 2"
#...
```

```bash
# In terminal running listener_qos
[INFO] [1674572639.695289182] [listener_qos]: Best effort listener
[INFO] [1674572640.023684925] [listener_qos]: I heard: [Hello World: 0]
[INFO] [1674572641.023416691] [listener_qos]: I heard: [Hello World: 1]
[INFO] [1674572642.024505343] [listener_qos]: I heard: [Hello World: 2]
#...
```

### Set Parameters Callback
#### [Before]
Run `ros2 param get /set_parameters_callback param1` should print the following to the terminal:
```bash
Double value is: 0.0
```
Run `ros2 param get /set_parameters_callback param2` should print the following to the terminal:
```bash
Double value is 0.0
```
#### [Change]
Run `ros2 param set /set_parameters_callback param1 28.0` should print the following to the terminal:
```bash
Set parameter successful
```
#### [After]
Run `ros2 param get /set_parameters_callback param1` should print the following to the terminal:
```bash
Double value is: 28.0
```
Run `ros2 param get /set_parameters_callback param2` should print the following to the terminal:
```bash
Double value is 4.0
```

### Asynchronous Parameter Client

Running `ros2 run demo_nodes_cpp parameter_blackboard` should print the output to the terminal similar to what is shown below:

```bash
# In terminal running async_param_client
[INFO] [1674574042.977327137] [async_param_client]: Setting parameters:
[INFO] [1674574042.978217825] [async_param_client]:     int_parameter:
[INFO] [1674574042.978474500] [async_param_client]:         successful: True
[INFO] [1674574042.978718655] [async_param_client]:     bool_parameter:
[INFO] [1674574042.978960532] [async_param_client]:         successful: True
[INFO] [1674574042.979200840] [async_param_client]:     string_parameter:
[INFO] [1674574042.979446692] [async_param_client]:         successful: True
[INFO] [1674574042.979687030] [async_param_client]: Listing Parameters:
[INFO] [1674574042.980229123] [async_param_client]:     - bool_parameter
[INFO] [1674574042.980476617] [async_param_client]:     - int_parameter
[INFO] [1674574042.980717248] [async_param_client]:     - string_parameter
[INFO] [1674574042.980956813] [async_param_client]: Getting parameters:
[INFO] [1674574042.981544032] [async_param_client]:     - int_parameter: 10
[INFO] [1674574042.981794832] [async_param_client]:     - bool_parameter: False
[INFO] [1674574042.982038493] [async_param_client]:     - string_parameter: Fee Fi Fo Fum
[INFO] [1674574042.982281892] [async_param_client]: Loading parameters:
[INFO] [1674574042.987160462] [async_param_client]:     other_int_parameter:
[INFO] [1674574042.987431457] [async_param_client]:         successful: True
[INFO] [1674574042.987679192] [async_param_client]:         value: 0
[INFO] [1674574042.987920948] [async_param_client]:     int_parameter:
[INFO] [1674574042.988162645] [async_param_client]:         successful: True
[INFO] [1674574042.988402691] [async_param_client]:         value: 12
[INFO] [1674574042.988643394] [async_param_client]:     string_parameter:
[INFO] [1674574042.988884034] [async_param_client]:         successful: True
[INFO] [1674574042.989125729] [async_param_client]:         value: I smell the blood of an Englishman
[INFO] [1674574042.989365771] [async_param_client]:     other_string_parameter:
[INFO] [1674574042.989605537] [async_param_client]:         successful: True
[INFO] [1674574042.989847066] [async_param_client]:         value: One fish, two fish, Red fish, blue fish
[INFO] [1674574042.990086215] [async_param_client]:     bool_parameter:
[INFO] [1674574042.990325361] [async_param_client]:         successful: True
[INFO] [1674574042.990565005] [async_param_client]:         value: False
[INFO] [1674574042.990804701] [async_param_client]: Deleting parameters:
[INFO] [1674574042.991497679] [async_param_client]:     other_int_parameter:
[INFO] [1674574042.991746545] [async_param_client]:         successful: True
[INFO] [1674574042.991986980] [async_param_client]:         reason:
[INFO] [1674574042.992226951] [async_param_client]:     other_string_parameter:
[INFO] [1674574042.992465274] [async_param_client]:         successful: True
[INFO] [1674574042.992703259] [async_param_client]:         reason:
[INFO] [1674574042.992941566] [async_param_client]:     string_parameter:
[INFO] [1674574042.993179508] [async_param_client]:         successful: True
[INFO] [1674574042.993416599] [async_param_client]:         reason:
```

### Serialized Subscriber

Running `ros2 run demo_nodes_py talker` should print output to the terminal similar to the following:

```bash
# In terminal running listener_serialized
[INFO] [1674574499.556053634] [serialized_subscriber]: I heard: "b'\x00\x01\x00\x00\x0f\x00\x00\x00Hello World: 0\x00\x00'"
[INFO] [1674574500.506343148] [serialized_subscriber]: I heard: "b'\x00\x01\x00\x00\x0f\x00\x00\x00Hello World: 1\x00\x00'"
[INFO] [1674574501.508214693] [serialized_subscriber]: I heard: "b'\x00\x01\x00\x00\x0f\x00\x00\x00Hello World: 2\x00\x00'"
```

## **References**

1. [About Quality of Service Settings](https://docs.ros.org/en/rolling/Concepts/About-Quality-of-Service-Settings.html)
