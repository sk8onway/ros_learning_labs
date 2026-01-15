# 📘 ROS 2 QoS — Revision Notes (Programming-Driven)

These notes are based on experiments, not theory.

---

## 1️⃣ What is QoS in ROS 2?

QoS (Quality of Service) defines how messages are delivered between nodes.

ROS 2 uses DDS, which treats communication as a contract:

* Publisher offers
* Subscriber requests
* If incompatible → no communication

📌 ROS 1 hid this. ROS 2 forces you to think.

---

## 2️⃣ Why QoS matters (real reason)

QoS only becomes visible when:

* Subscriber is slow
* Network / CPU is stressed
* Nodes start at different times
* Buffers overflow

👉 Under ideal conditions, all QoS looks the same.

---

## 3️⃣ Reliability Policy

### BEST_EFFORT
```python
ReliabilityPolicy.BEST_EFFORT
```

**Observed behavior**

* Messages can be dropped
* No retries
* Lower latency
* Subscriber sees gaps in message sequence

**Best for**

* Sensors (`/scan`, `/image_raw`)
* State streams where latest value matters

### RELIABLE
```python
ReliabilityPolicy.RELIABLE
```

**Observed behavior**

* No message loss
* Retries + acknowledgements
* Increased latency
* Subscriber may lag behind

**Best for**

* Commands
* Maps
* Critical state

### 🔑 Reliability Compatibility Rule

| Publisher    | Subscriber   | Result |
|--------------|--------------|--------|
| RELIABLE     | RELIABLE     | ✅     |
| RELIABLE     | BEST_EFFORT  | ✅     |
| BEST_EFFORT  | BEST_EFFORT  | ✅     |
| BEST_EFFORT  | RELIABLE     | ❌     |

➡️ ROS 2 blocks communication rather than silently violating guarantees.

---

## 4️⃣ Queue Depth (History Depth)
```python
QoSProfile(depth=N)
```

### What depth actually is

* Size of the message buffer
* NOT flow control
* NOT performance improvement

### depth = 1

* Old messages dropped immediately
* Subscriber always gets latest data
* No growing delay

**Used for:**

* Control loops
* Robot state
* `/tf`

### depth = 10 (or higher)

* Messages buffered
* Subscriber processes old data
* Delay grows if subscriber is slow

**Used for:**

* Logging
* Diagnostics
* Data replay

### 🔥 Critical Insight (from your experiment)

If publisher rate > subscriber rate, subscriber will never catch up, regardless of depth.

Depth only controls what gets dropped, not whether drops happen.

---

## 5️⃣ Durability Policy (Late Joiners)

Durability answers: **"Does the publisher remember old messages?"**

### VOLATILE (default)
```python
DurabilityPolicy.VOLATILE
```

* No storage
* Late subscribers get nothing
* Real-time only

### TRANSIENT_LOCAL
```python
DurabilityPolicy.TRANSIENT_LOCAL
```

* Publisher stores last message(s)
* Late subscriber immediately receives data
* Equivalent to ROS 1 latched topics

### 🔑 Durability Compatibility Rule

| Publisher        | Subscriber       | Result |
|------------------|------------------|--------|
| VOLATILE         | VOLATILE         | ✅     |
| TRANSIENT_LOCAL  | TRANSIENT_LOCAL  | ✅     |
| TRANSIENT_LOCAL  | VOLATILE         | ✅     |
| VOLATILE         | TRANSIENT_LOCAL  | ❌     |

➡️ Durability must be set on publisher to matter.

---

## 6️⃣ Key Takeaways (Exam / Viva Ready)

* QoS differences appear only under stress
* Depth ≠ ability to keep up
* Reliability trades latency for correctness
* Durability enables late joiners
* ROS 2 prefers no communication over wrong communication

---

## 7️⃣ One-Line Mental Model (Gold)

**QoS does not fix overload — it defines how overload is handled.**

---