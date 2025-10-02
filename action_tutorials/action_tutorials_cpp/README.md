# Action Server

In the constructor for `FibonacciActionServer`, an action server is created with callbacks that are called when a goal is received, when the goal is cancelled, and when the goal is accepted:

```cpp
this->action_server_ = rclcpp_action::create_server<Fibonacci>(
  this,
  "fibonacci",
  handle_goal,
  handle_cancel,
  handle_accepted);
```

The `handle_goal` callback is called whenever a goal is sent to the action server by an action client.
In the example code, the goal is accepted as long as the order is less than or equal to 46, otherwise it is rejected.
This is to prevent potential [integer overflow](https://en.wikipedia.org/wiki/Integer_overflow):

```cpp
if (goal->order > 46) {
  return rclcpp_action::GoalResponse::REJECT;
}
return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
```

The `handle_cancelled` callback is called whenever an action client requests to cancel the goal being executed.
In this case, the goal cancel request is always accepted.

The `handle_accepted` callback is called following the action server's acceptance of a goal. In this example, a thread is started to execute the goal:

```cpp
auto execute_in_thread = [this, goal_handle](){return this->execute(goal_handle);};
std::thread{execute_in_thread}.detach();
```

The execution thread calculates the Fibonacci sequence up to *order* and publishes partial sequences as feedback as each item is added to the sequence.

A `rclcpp::Rate` object is used to sleep between the calculation of each item in order to represent a long-running task.

When execution is complete, the full sequence is returned to the action client.
If the goal is cancelled during execution, no result is returned, however the caller may have received partial sequences as feedback up until cancelling.

# Action Client

In the constructor for `FibonacciActionClient`, and action client for the `fibonacci` action is created:

```cpp
this->client_ptr_ = rclcpp_action::create_client<Fibonacci>(
      ...
      "fibonacci");
```

A goal of type `Fibonacci` is created with order 10.
The goal is sent asynchronously with callbacks registered for the goal response, the feedback, and the goal result:

```cpp
auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
send_goal_options.goal_response_callback = [this](
  const GoalHandleFibonacci::SharedPtr & goal_handle)
{...};
send_goal_options.feedback_callback = [this](
  GoalHandleFibonacci::SharedPtr,
  const std::shared_ptr<const Fibonacci::Feedback> feedback)
{...};
send_goal_options.result_callback = [this](
  const GoalHandleFibonacci::WrappedResult & result)
{...};
this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
```

There are three types of callback functions:

- The `goal_response_callback` is called when the action server accepts or rejects the goal.
- The `feedback_callback` is called whenever the action server sends goal execution feedback.
- The `goal_result_callback` is called when the action server is finished executing the goal and returns the result of the goal which is the full or partial Fibonacci sequence.
