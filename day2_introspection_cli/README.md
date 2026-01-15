# ROS 2 Learning Lab — Day 2  
**Introspection & CLI Tools**

This lab focuses on understanding a running ROS 2 system using command-line introspection tools.  
The goal was to observe the **ROS graph** (nodes, topics, connections) and debug communication issues without modifying code.

## Concepts Covered
- ROS graph (nodes, topics, message flow)
- Application topics vs system topics
- Publishers, subscribers, and timers
- Runtime introspection vs source inspection

## CLI Tools Used
```bash
ros2 node list
ros2 node info /node_name
ros2 topic list
ros2 topic info /topic_name
ros2 topic echo /topic_name
ros2 interface show pkg/msg/MessageType
```

## Key Learnings

- ROS introspection reflects current state only

- Topics are broadcast (multiple subscribers allowed)

- /rosout and /parameter_events are system topics

- ros2 topic info is critical for debugging connectivity

- Message fields should always be inspected, not guessed

## Setup

- Introspection was performed on the Day-1 pub/sub system:

- number_publisher → publishes integers on /numbers

- number_subscriber → subscribes to /numbers

## Summary

Day-2 made ROS transparent by teaching how to inspect and debug live systems using CLI tools, an essential skill for real-world robotics.