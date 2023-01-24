## **What Is This?**

This demo ...

This ROS 2 package consists of the following nodes:

1. `dlopen_composition`
2. `linktime_composition`
3. `manual_composition`

## **Build**

Run the commands below to build the ROS 2 package:

```bash
colcon build --packages-up-to composition
```

## **Run**

### Manual Composition

```bash
ros2 run 
```

### DlOpen Composition


```bash

```

### Linktime Composition


```bash

```


## **Verify**

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
```

### DlOpen Composition

When executed correctly, strings should be printed to terminal similar to what is shown below:

```bash

```

### Linktime Composition

```bash

```

## **FAQ**

WIP

## **References**

1. [Composing multiple nodes in a single process](https://docs.ros.org/en/rolling/Tutorials/Intermediate/Composition.html)
2. [About Composition](https://docs.ros.org/en/rolling/Concepts/About-Composition.html#about-composition)
3. []()
