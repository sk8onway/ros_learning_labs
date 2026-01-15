# ROS 2 QoS Publisher–Subscriber Lab

This lab demonstrates the effect of ROS 2 QoS policies through a Python-based publisher–subscriber example.

## Concepts Covered

- Reliability (BEST_EFFORT vs RELIABLE)
- Queue depth (1 vs 10)
- Durability (VOLATILE vs TRANSIENT_LOCAL)
- QoS compatibility rules
- Behavior under subscriber delay and CPU stress

---

## How to Run

### Publisher
```bash
ros2 run joke_teller qos_joke_publisher
```

### Subscriber
```bash
ros2 run joke_teller joke_subscriber
```

---

## Experiments

- Slow subscriber to observe depth effects
- Stress CPU to observe reliability behavior
- Restart subscriber to test durability

---

## Observations

- BEST_EFFORT drops messages under load but maintains low latency
- RELIABLE ensures delivery but increases delay
- depth=1 keeps data fresh, depth=10 preserves history
- TRANSIENT_LOCAL allows late subscribers to receive last message
- QoS incompatibility prevents communication entirely

---

## Conclusion

QoS defines communication contracts in ROS 2 and becomes critical under non-ideal conditions such as overload, delays, or late joins.

---