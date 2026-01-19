# 📝 ROS 2 Services – Revision Notes & Deep Learnings

This document captures **conceptual clarity, caveats, and lessons learned** while implementing ROS 2 services in Python.

Use this as **revision material** before exams, interviews, or real projects.

---

## 🧠 Core Concepts (Must Know)

### What creates a service?

- A **service server**
- NOT the `.srv` file
- NOT a client
- NOT a CLI call

> No server → no service.

---

### Can a server exist without a client?

✅ Yes  
It simply waits.

### Can a service exist without a server?

❌ No

---

## 🧩 Two Ways to Create a Service

### 1️⃣ Using an existing service type (`AddTwoInts`)

Example:
```python
from example_interfaces.srv import AddTwoInts
```

#### ✅ Pros

- Zero interface setup
- No rosidl
- No CMakeLists.txt
- Fast prototyping
- Great for learning service mechanics

#### ❌ Cons

- Semantics may not match your use-case
- Field names may be misleading (sum used as product)
- Not suitable for real projects

#### When to use

- Learning
- Demos
- Debugging
- Quick validation

---

### 2️⃣ Using a custom `.srv` interface

Example:
```python
from joke_teller_interfaces.srv import MultiplyTwoInts
```

#### ✅ Pros

- Correct semantics
- Clean API
- Scales to real systems
- Industry-standard practice

#### ❌ Cons (IMPORTANT)

Creating a custom service introduces real complexity:

- Requires a separate interfaces package
- Requires ament_cmake
- Requires rosidl generators
- Requires clean rebuilds
- Easy to break if mixed with Python packaging

**Most beginner ROS issues start here.**

---

## 🔥 Why custom services cause problems (and why that's okay)

### Common mistakes

- Mixing `.srv` and Python nodes in one package
- Using `ament_python` with rosidl
- Forgetting to clean `build/install/log`
- Expecting generated Python files in the wrong directory
- Forgetting to source the workspace

### Correct pattern

**Interfaces are infrastructure.**  
**Nodes are behavior.**  
**Never mix them.**

---

## 🛠️ Build System Rules (Non-negotiable)

### Safe to rebuild normally
```text
Python files
setup.py
```

### Requires clean rebuild
```text
CMakeLists.txt
package.xml
.srv files
dependencies
```
```bash
rm -rf build install log
```

---

## 🐛 High-value Bugs Faced (and Fixes)

### ❌ Service Python module not found

**Cause:**  
rosidl-generated code not installed due to wrong build type

**Fix:**  
Separate `*_interfaces` package with `ament_cmake`

---

### ❌ `ros2 run` shows no executables

**Causes:**

- Missing `resource/<pkgname>`
- Missing `__init__.py`
- Missing or incorrect `setup.cfg`
- Environment not sourced

---

### ❌ Double Python packaging error

**Cause:**  
Using both `ament_cmake_python` and setuptools

**Fix:**  
One package → one build system

---

## 🔁 Environment Rules (Tattoo-worthy)

Every new terminal:
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

**90% of ROS "bugs" disappear here.**

---

## 🧠 Mental Models (Exam Gold)

- **Topics** → continuous data stream
- **Services** → request–response (short-lived)
- **Actions** → long-running tasks with feedback
- CLI calls are temporary clients
- Client nodes are real system components

---

## 🧪 Debugging Checklist
```bash
ros2 pkg list
ros2 service list
ros2 service type /multiply_two_ints
ros2 interface show joke_teller_interfaces/srv/MultiplyTwoInts
ros2 pkg executables joke_teller
```

If these work, your system is correct.

---

## 🎯 Final Takeaway

**ROS 2 punishes architectural mistakes, not logical ones.**

Once the architecture is correct, services become simple and reliable.

Understanding why things break is more valuable than just making them work.

---