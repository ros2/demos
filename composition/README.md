## What Is This?

This demo provides examples of three different ways to use the **rclcpp_components** API to compose multiple nodes in a single process.

This ROS 2 package consists of the following demo applications:

1. `dlopen_composition`
2. `linktime_composition`
3. `manual_composition`

## Build

Run the commands below to build the ROS 2 package:

```bash
colcon build --packages-up-to composition
```

## Run

### Manual Composition

Running `manual_composition` compiles an executable that runs the following 4 components:

- **Talker**: A ROS 2 component that publishes a string
- **Listener**: A ROS 2 component that prints the received string from **Talker**
- **Server**: A ROS 2 component that adds two integers and outputs its result to **Client**
- **Client**: A ROS 2 component that sends two integers to **Server** and prints the received result from **Server**

```bash
ros2 run composition manual_composition
```

### DlOpen Composition

This runs `dlopen_composition` which is an alternative to run-time composition by creating a generic container process and explicitly passing the libraries to load without using ROS interfaces.

The process will open each library and create one instance of each “rclcpp::Node” class in the library.

```bash
ros2 run composition dlopen_composition `ros2 pkg prefix composition`/lib/libtalker_component.so `ros2 pkg prefix composition`/lib/liblistener_component.so
```

### Linktime Composition

Similar to previous, this runs `linktime_composition` which **links all classes from libraries** that are registered under the **library_path** with the **linker**.

```bash
ros2 run composition linktime_composition
```

### Composition Using Launch Actions

Rather than using the command line tool to run each composition, we can **automate this action** with `ros2 launch` functionality:

```bash
ros2 launch composition composition_demo_launch.py
```

## Verify

### Manual Composition

When executed correctly, strings should be printed to terminal similar to what is shown below:

```bash
[INFO] [1674528188.026468320] [talker]: Publishing: 'Hello World: 1'
[INFO] [1674528188.027043857] [listener]: I heard: [Hello World: 1]
[INFO] [1674528189.026414368] [talker]: Publishing: 'Hello World: 2'
[INFO] [1674528189.026742015] [listener]: I heard: [Hello World: 2]
[INFO] [1674528189.032512995] [Server]: Incoming request: [a: 2, b: 3]
[INFO] [1674528189.032815843] [Client]: Got result: [5]
[INFO] [1674528190.026455807] [talker]: Publishing: 'Hello World: 3'
[INFO] [1674528190.026795770] [listener]: I heard: [Hello World: 3]
[INFO] [1674528191.026457639] [talker]: Publishing: 'Hello World: 4'
[INFO] [1674528191.026801926] [listener]: I heard: [Hello World: 4]
[INFO] [1674528191.032377264] [Server]: Incoming request: [a: 2, b: 3]
[INFO] [1674528191.032604427] [Client]: Got result: [5]
[INFO] [1674528192.026428269] [talker]: Publishing: 'Hello World: 5'
[INFO] [1674528192.026537974] [listener]: I heard: [Hello World: 5]
[INFO] [1674528193.026437034] [talker]: Publishing: 'Hello World: 6'
[INFO] [1674528193.026767708] [listener]: I heard: [Hello World: 6]
[INFO] [1674528193.032377748] [Server]: Incoming request: [a: 2, b: 3]
[INFO] [1674528193.032603036] [Client]: Got result: [5]
#...
```

:warning:
> Note that manually-composed components **will not be reflected in the `ros2 component list`** command line tool output.

### DlOpen Composition

When executed correctly, strings should be printed to terminal similar to what is shown below:

```bash
INFO] [1674529118.496557668] [dlopen_composition]: Load library /opt/ros/rolling/lib/libtalker_component.so
[INFO] [1674529118.496774575] [dlopen_composition]: Instantiate class rclcpp_components::NodeFactoryTemplate<composition::Talker>
[INFO] [1674529118.503388909] [dlopen_composition]: Load library /opt/ros/rolling/lib/liblistener_component.so
[INFO] [1674529118.503739855] [dlopen_composition]: Instantiate class rclcpp_components::NodeFactoryTemplate<composition::Listener>
[INFO] [1674529119.503505873] [talker]: Publishing: 'Hello World: 1'
[INFO] [1674529119.503770137] [listener]: I heard: [Hello World: 1]
[INFO] [1674529120.503572362] [talker]: Publishing: 'Hello World: 2'
[INFO] [1674529120.503888374] [listener]: I heard: [Hello World: 2]
[INFO] [1674529121.503503459] [talker]: Publishing: 'Hello World: 3'
[INFO] [1674529121.503628269] [listener]: I heard: [Hello World: 3]
[INFO] [1674529122.503557862] [talker]: Publishing: 'Hello World: 4'
[INFO] [1674529122.503894772] [listener]: I heard: [Hello World: 4]
[INFO] [1674529123.503574524] [talker]: Publishing: 'Hello World: 5'
[INFO] [1674529123.503884894] [listener]: I heard: [Hello World: 5]
#...
```

:warning:
> Note that dlopen-composed components **will not be reflected in the `ros2 component list`** command line tool output.


### Linktime Composition

When executed correctly, strings should be printed to terminal similar to what is shown below:

```bash
[INFO] [1674528568.091949637] [linktime_composition]: Load library
[INFO] [1674528568.091995119] [linktime_composition]: Instantiate class rclcpp_components::NodeFactoryTemplate<composition::Client>
[INFO] [1674528568.098833910] [linktime_composition]: Instantiate class rclcpp_components::NodeFactoryTemplate<composition::Listener>
[INFO] [1674528568.100669644] [linktime_composition]: Instantiate class rclcpp_components::NodeFactoryTemplate<composition::Server>
[INFO] [1674528568.102665704] [linktime_composition]: Instantiate class rclcpp_components::NodeFactoryTemplate<composition::Talker>
[INFO] [1674528569.104717098] [talker]: Publishing: 'Hello World: 1'
[INFO] [1674528569.105206993] [listener]: I heard: [Hello World: 1]
[INFO] [1674528570.099206827] [Server]: Incoming request: [a: 2, b: 3]
[INFO] [1674528570.099376432] [Client]: Got result: [5]
[INFO] [1674528570.104656875] [talker]: Publishing: 'Hello World: 2'
[INFO] [1674528570.105069514] [listener]: I heard: [Hello World: 2]
[INFO] [1674528571.104710545] [talker]: Publishing: 'Hello World: 3'
[INFO] [1674528571.105150094] [listener]: I heard: [Hello World: 3]
[INFO] [1674528572.099350955] [Server]: Incoming request: [a: 2, b: 3]
[INFO] [1674528572.099628903] [Client]: Got result: [5]
[INFO] [1674528572.104631322] [talker]: Publishing: 'Hello World: 4'
[INFO] [1674528572.104911174] [listener]: I heard: [Hello World: 4]
[INFO] [1674528573.104596009] [talker]: Publishing: 'Hello World: 5'
[INFO] [1674528573.104751214] [listener]: I heard: [Hello World: 5]
#...
```

:warning:
> Note that linktime-composed components **will not be reflected in the `ros2 component list`** command line tool output.


## FAQ

`Q`: Why use node composition?

`A`: Node composition avoids the overhead of marshalling and unmarshaling messages by allowing nodes to be instantiated within the same process.

## References

1. [Composing multiple nodes in a single process](https://docs.ros.org/en/rolling/Tutorials/Intermediate/Composition.html)
2. [About Composition](https://docs.ros.org/en/rolling/Concepts/About-Composition.html#about-composition)
