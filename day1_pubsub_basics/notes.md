ROS 2 Learning Lab — Day 1 Notes

Core Basics + Debugging Mindset

These notes document my Day-1 hands-on learning of ROS 2 fundamentals.
The focus was not only on making things work, but on understanding how ROS 2 actually executes nodes, how Python packaging interacts with ROS, and how to debug real ROS errors systematically.

1. What ROS 2 Is (Mental Model)

ROS 2 is a framework, not:

an operating system

a programming language

a simulator

What ROS 2 provides

A communication layer for robot software

Tools to run, inspect, and debug distributed programs

Conventions for structuring robotic systems

Key idea

ROS 2 helps many small programs (nodes) communicate reliably.

2. Nodes

A node is a running process.

Each node should ideally perform one responsibility.

Examples:

Camera node → publishes images

Controller node → subscribes to data and decides actions

Why many nodes?

Fault isolation

Easier debugging

Reusability

3. Topics

A topic is a named communication channel.

Nodes never communicate directly; they communicate via topics.

Examples:

/numbers
/chatter
/cmd_vel


Topic properties:

One-way (publisher → subscriber)

Typed (message type must match)

Asynchronous

4. Publishers and Subscribers
Publisher

Sends messages on a topic

Does not care who receives them

Subscriber

Listens to a topic

Reacts using a callback function

Critical Rule

Topic name AND message type must match exactly.

If they don’t:

No error

No warning

No data flow (silent failure by design)

5. Message Types (std_msgs)

std_msgs contains standard message definitions:

Int32

String

Float64

Correct import:

from std_msgs.msg import Int32


Incorrect (runtime error):

from std_msgs.msgs import Int32


Important: ROS message modules are always singular → msg, not msgs.

6. QoS (Quality of Service) — Meaning Only

Example:

create_publisher(Int32, 'numbers', 10)


10 = queue depth

Number of messages ROS buffers if subscribers are slow

QoS is not:

publish frequency

number of subscribers

7. Python ROS 2 Package Structure

Created using:

ros2 pkg create my_pubsub --build-type ament_python --dependencies rclpy std_msgs


Resulting structure:

my_pubsub/
├── my_pubsub/          ← source code (acts like src/)
│   ├── __init__.py
│   ├── number_publisher.py
│   └── number_subscriber.py
├── setup.py
├── setup.cfg
├── package.xml


Note: Python ROS packages do not use a src/ directory.
The inner folder is the source.

8. Canonical Node Code Structure
Publisher Example
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class NumberPublisher(Node):
    def __init__(self):
        super().__init__('number_publisher')

        self.publisher_ = self.create_publisher(Int32, 'numbers', 10)
        self.counter_ = 0
        self.timer_ = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = Int32()
        msg.data = self.counter_
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.counter_ += 1

def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

9. Why main() Is Outside the Class

Incorrect:

class MyNode(Node):
    def main(self):
        ...


Correct:

def main():
    ...


Reason:

ROS executes a module-level function

setup.py explicitly calls module:function

ROS does not auto-instantiate classes

Separation of concerns

Class → node behavior

main() → lifecycle management

10. setup.py vs setup.cfg
setup.py

Defines executable entry points

Controls how ROS runs nodes

entry_points={
    'console_scripts': [
        'number_publisher = my_pubsub.number_publisher:main',
        'number_subscriber = my_pubsub.number_subscriber:main',
    ],
},

setup.cfg

Metadata only

Package name, version, install rules

Never duplicate logic across both files.

11. Why ros2 run my_pubsub number_publisher

Syntax:

ros2 run <package_name> <executable_name>


Why package name is mandatory:

Multiple packages may contain the same executable name

ROS avoids ambiguity by design

ROS does not search globally

This will never work:

ros2 run number_publisher

12. Executable Permissions (chmod +x)

chmod +x is not required for ros2 run

ROS runs executables via entry points, not directly from source files

Executable permission is only required for:

./number_publisher.py


File highlighting in ls is a Linux feature, not a ROS requirement.

13. spin(), destroy_node(), shutdown()

rclpy.spin(node)

Blocks execution

Processes callbacks (timers, subscribers)

node.destroy_node()

Cleans up publishers, subscribers, timers

rclpy.shutdown()

Shuts down ROS client library

Releases DDS resources

Cleanup happens after spin() exits (Ctrl+C).

14. Subscriber Logic

Subscriber callback runs only when:

spin() is active

a message arrives

self.subscription_ = self.create_subscription(
    Int32,
    'numbers',
    self.listener_callback,
    10
)

def listener_callback(self, msg):
    self.get_logger().info(f'Received: {msg.data}')

15. Source vs Installed Code (Critical Insight)

ROS does not run nodes from src/.

It runs executables from:

install/my_pubsub/lib/my_pubsub/


Debugging tip:

ls ~/ros2_ws/install/my_pubsub/lib/my_pubsub


If an executable isn’t there, ROS cannot run it.

16. Python List Comma Bug (Real Issue Faced)

Mistake:

'a'
'b'


Python silently concatenates:

'ab'


Result:

Broken ROS executables

No syntax error

Hard-to-spot bug

Always double-check commas in entry_points.

17. Debugging Mindset — Reading Tracebacks

Correct order:

Read the last line (actual error)

Identify error type

Understand what was expected but missing

Examples encountered:

Missing main() → wrong function scope

.publisher() vs .publish() → method typo

std_msgs.msgs → incorrect import

Most ROS issues were Python runtime issues, not ROS bugs.

18. General Debugging Algorithm

Read last traceback line

Identify error category

Fix one thing only

Rebuild and re-source

Re-run

Avoid:

shotgun debugging

reinstalling ROS for logic errors

Final Takeaway

By the end of Day-1, I understood:

How ROS 2 executes nodes

How Python packaging integrates with ROS

Why many ROS errors are runtime-only

How to debug ROS systems systematically

This was not just learning ROS commands —
it was learning how to think while working with ROS.