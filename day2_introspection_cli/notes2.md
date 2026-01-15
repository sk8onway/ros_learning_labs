# ROS 2 Learning Lab — Day 2 Notes

Introspection, ROS Graph & CLI Debugging

These notes cover Day-2 of my ROS 2 learning, focused on introspecting a running ROS system using CLI tools.
The goal was to move from “running nodes” to understanding and debugging the live ROS graph.

1. The ROS Graph (Core Mental Model)

A running ROS system is represented internally as a graph consisting of:

Nodes

Topics

Message types

Connections between nodes

Example graph from Day-1 pub/sub:

[number_publisher] --(/numbers : Int32)--> [number_subscriber]


Key idea

ROS is not just programs running — it is a live communication graph.

All introspection commands query the current state of this graph.

2. Listing Active Nodes
Command
ros2 node list

What it shows

Names of currently running nodes

Node names, not package names or filenames

Example output:

/number_publisher
/number_subscriber

Debugging rule

If a node does not appear here, it is not running.

This is the first command to run when debugging.

3. Inspecting a Node
Command
ros2 node info /number_publisher

What it shows

Publishers

Subscribers

Services

Parameters

This tells you what a node is capable of, not its code.

Important detail

The leading / in the node name matters

ros2 node info number_publisher ❌

ros2 node info /number_publisher ✅

4. Timers vs Subscribers (Important Distinction)

In number_publisher:

✅ Publisher → /numbers

✅ Timer → triggers callbacks internally

❌ No subscribers

Timers are NOT subscribers.

Concept	Source of trigger
Subscriber	External data (topic)
Timer	Internal clock
5. System Topics (/rosout and /parameter_events)

While inspecting topics and nodes, two system topics always appear.

/rosout

Used for logging

Every node publishes log messages here

Created automatically by ROS

/parameter_events

Used to broadcast parameter changes

Message type: rcl_interfaces/msg/ParameterEvent

Exists even if you don’t use parameters explicitly

Rule

If you didn’t design it, and it sounds generic, it’s probably a system topic.

For application logic, these can be ignored.

6. Listing Topics
Command
ros2 topic list

Typical output (Day-1 system)
/numbers
/rosout
/parameter_events


Topics fall into two categories:

Application topics

/numbers

/cmd_vel

/scan

System topics

/rosout

/parameter_events

7. Echoing a Topic (Live Data Inspection)
Command
ros2 topic echo /numbers

What it does

Subscribes to the topic

Prints every message published

Does NOT interfere with existing subscribers

Important concept

Topics are broadcast, not consumed.

Multiple subscribers can listen simultaneously:

your subscriber node

ros2 topic echo

debugging tools

8. Inspecting Topic Connectivity
Command
ros2 topic info /numbers

Output fields
Type: std_msgs/msg/Int32
Publisher count: 1
Subscription count: 2

How to read this
Message type

Must match exactly between publisher and subscriber

Mismatch causes silent failure

Publisher count

0 → no active publisher

1+ → topic is being published

Subscription count

CLI tools count as subscribers

Helps confirm who is listening

Debugging rule

If data isn’t flowing, check publisher count first.

9. Inspecting Message Definitions
Command
ros2 interface show std_msgs/msg/Int32

Output
int32 data

Meaning

Message has one field

Field name: data

Field type: int32

This explains why code uses:

msg.data

Rule

Never guess message fields.
Always inspect the interface.

10. Full Introspection Workflow (Very Important)

A systematic ROS debugging loop:

ros2 node list → who is running

ros2 node info /node → what the node does

ros2 topic list → what topics exist

ros2 topic info /topic → who is connected

ros2 topic echo /topic → live data

ros2 interface show → message structure

This workflow replaces guessing with verification.

11. Key Debugging Lessons from Day-2

ROS introspection tools show current state only

Nodes must be running to inspect them

CLI tools are real subscribers

System topics are normal and expected

Connectivity issues are usually existence issues, not logic bugs

## Final Takeaway

Day-2 removed the “magic” from ROS.

I can now:

See the live ROS graph

Verify whether nodes and topics exist

Debug communication issues using CLI tools

Understand message structures before writing code

This makes ROS observable and debuggable, not opaque.