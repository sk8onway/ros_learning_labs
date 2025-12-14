# ROS 2 Learning Lab — Day 1

This repository documents my hands-on learning of ROS 2 core concepts.

## What I implemented
- Created a ROS 2 Python package using `ament_python`
- Implemented a publisher node that publishes integers periodically
- Implemented a subscriber node that receives and logs data
- Registered executables via `setup.py` entry points
- Ran nodes using `ros2 run`

## Concepts covered
- Nodes, topics, publishers, subscribers
- Message types (`std_msgs`)
- Timers and callbacks
- ROS 2 execution model (`spin`)
- Package structure and installation
- Difference between source code and installed artifacts

## Debugging lessons learned
- Interpreting Python tracebacks in ROS 2
- Common entry-point mistakes in `setup.py`
- Runtime vs build-time errors
- Silent Python bugs (missing commas, wrong imports)
- How ROS resolves executables from `install/`

## Why this matters
This lab helped me understand how ROS 2 actually executes nodes,
not just how to follow tutorials.
