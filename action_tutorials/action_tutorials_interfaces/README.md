# Action Tutorials ROS 2 Interface

This tutorial defines the Fibonacci action for use with the action tutorials.
There are three parts of the action:

- The goal contains an *order* field which determines the length of the returned Fibonacci sequence.
For example, order 2 should return sequence [0, 1] and order 5 should return sequence [0, 1, 1, 2, 3].
- The feedback consists of a partial sequence that is returned as the Fibonacci sequence is calculated.
For example, for order 5 at some point the partial sequence [0, 1, 1] will be returned.
- The result consists of the complete Fibonacci sequence (ex. [0, 1, 1, 2, ..., 165580141]).
