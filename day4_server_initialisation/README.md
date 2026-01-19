# 🧠 ROS 2 Services – Server & Client (Learning Lab)

This lab documents a **realistic, end-to-end journey** of implementing ROS 2 services in Python.
It goes beyond "happy path" tutorials and captures **actual pitfalls, debugging steps, and architectural decisions** encountered while learning ROS 2 services properly.

The goal of this lab is not just to make a service work, but to understand:
- *what creates a service*
- *how clients interact with it*
- *why build systems matter*
- *and where things usually break*

---

## 📌 What this lab covers

- ROS 2 **Services fundamentals**
- Difference between:
  - service server
  - service client node
  - CLI service call
- Two ways to create a service:
  - using existing service types (`AddTwoInts`)
  - using a custom `.srv` interface
- Correct **package architecture**
- Writing:
  - service server node
  - service client node
- Understanding:
  - `ament_python`
  - `ament_cmake`
  - rosidl interface generation
- Debugging real-world ROS errors
- Best practices for ROS workspaces

---

## 🗂️ Workspace Structure (Canonical)
```text
ros2_ws/
├── src/
│   ├── joke_teller_interfaces/     # Interfaces ONLY (ament_cmake)
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   └── srv/
│   │       └── MultiplyTwoInts.srv
│   │
│   └── joke_teller/                # Python nodes ONLY (ament_python)
│       ├── package.xml
│       ├── setup.py
│       ├── setup.cfg
│       ├── resource/
│       │   └── joke_teller
│       └── joke_teller/
│           ├── __init__.py
│           ├── multiply_server.py
│           ├── multiply_server_custom.py
│           └── multiply_server_client.py
│
├── build/
├── install/
└── log/
```

---

## 🧾 Service Definition

`MultiplyTwoInts.srv`
```
int64 a
int64 b
---
int64 product
```

This file defines the data contract, not the service itself.
A service only exists at runtime when a server advertises it.

---

## 🟢 Service Server

The server:
* creates the service
* advertises it on the ROS graph
* waits for requests indefinitely

Example excerpt:
```python
self.create_service(
    MultiplyTwoInts,
    'multiply_two_ints',
    self.multiply_callback
)
```

---

## 🔵 Service Client

The client:
* does not create the service
* waits for a server
* sends requests programmatically
* handles responses asynchronously

Example excerpt:
```python
self.client = self.create_client(
    MultiplyTwoInts,
    'multiply_two_ints'
)
```

---

## 🧪 CLI Service Call vs Client Node

| Aspect           | CLI Service Call | Client Node |
|------------------|------------------|-------------|
| Lifetime         | One-shot         | Persistent  |
| Written in code  | ❌               | ✅          |
| Used in real robots | ❌            | ✅          |
| Async handling   | ❌               | ✅          |
| Retry / logic    | ❌               | ✅          |

CLI example:
```bash
ros2 service call /multiply_two_ints \
joke_teller_interfaces/srv/MultiplyTwoInts "{a: 6, b: 7}"
```

---

## 🚀 How to Run

### Terminal 1 – Start server
```bash
ros2 run joke_teller multiply_server_custom
```

### Terminal 2 – Run client
```bash
ros2 run joke_teller multiply_server_client
```

---

## 🎯 Takeaway

ROS 2 services are simple in concept, but strict in architecture. Most errors come from build-system misuse, not from Python code.

This lab focuses on understanding, not memorization.

For deeper explanations, pitfalls, and revision notes, see `notes.md`.

---