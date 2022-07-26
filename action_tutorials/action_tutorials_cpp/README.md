# Action Server

In the constructor for `FibonacciActionServer`, the action server is created with callbacks for when a goal is received, when the goal is cancelled and when the goal is accepted:

```
    this->action_server_ = rclcpp_action::create_server<Fibonacci>(
      ...
      "fibonacci",
      std::bind(&FibonacciActionServer::handle_goal, this, _1, _2),
      std::bind(&FibonacciActionServer::handle_cancel, this, _1),
      std::bind(&FibonacciActionServer::handle_accepted, this, _1));
```

In the `handle_goal` callback, the goal is accepted as long as the order is less than 46, otherwise it is rejected. This is to prevent potential integer overflow:
```
if (goal->order > 46) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
```

In the `handle_cancelled` callback, the request to cancel is unconditionally accepted.

In the `handle_accepted` callback, a thread is spun-up to execute the goal in the background:
```
    std::thread{std::bind(&FibonacciActionServer::execute, this, _1), goal_handle}.detach();
```

The `execute` method that is called by the thread loops for *order* times, calculates the next item in the Fibonacci sequence at each iteration and pushes it back to a partial sequence. The server sleeps for 1 second at each loop iteration to demonstrate a long-running task. 

The partial sequence is also published back to the client as feedback at each loop iteration. 

If the action is cancelled during execution, the partial sequence will be returned as the result:
```
if (goal_handle->is_canceling()) {
        result->sequence = sequence;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
```
Otherwise, the full sequence will be returned as the result at the end of the action execution.

# Action Client 
