# Action Server

In the constructor for `FibonacciActionServer`, an action server is created with a callback that is called when a goal is accepted.
```python
self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)
```

There are three types of callbacks:

- A `goal_callback` can optionally be added to conditionally accept or reject the goal, however, by default the goal is accepted.
- A `cancel_callback` can also optionally be added to conditionally accept or reject the cancel goal request, however, by default the cancel goal request is accepted.
- The `execute_callback` calculates the Fibonacci sequence up to *order* and publishes partial sequences as feedback as each item is added to the sequence.

The thread sleeps for 1 second between the calculation of each item in order to represent a long-running task.
When execution is complete, the full sequence is returned to the action client.

# Action Client

In the constructor for `FibonacciActionClient`, an action client for the `fibonacci` action is created:

```python
self._action_client = ActionClient(self, Fibonacci, 'fibonacci')
```

A goal of type `Fibonacci` is created with order 10.
The goal is sent asynchronously with callbacks registered for the goal response and the feedback:

```python
self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

self._send_goal_future.add_done_callback(self.goal_response_callback)
```

Within the `goal_response_callback`, if the goal is accepted, the goal result is requested asynchronously.
A callback is registered for receiving the result:
```python
self._get_result_future = goal_handle.get_result_async()

self._get_result_future.add_done_callback(self.get_result_callback)
```

There are two types of callbacks:

- The `feedback_callback` logs the partial sequences as they are received.
- The `get_result_callback` logs the full Fibonacci sequence.
