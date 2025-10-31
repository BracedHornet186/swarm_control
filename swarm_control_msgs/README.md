# swarm_control_msgs

A ROS2 package containing custom messages for `swarm_control` package.

## 📁 Package Overview

```
swarm_control_msgs/
├── msg/
│   ├── Info.msg
│   └── RBroadcast.msg
└── README.md
```


## Topic Overview (per bot)

| Topic | Type | Publisher | Description |
|--------|------|------------|--------------|
| `/bot_i/info` | `swarm_control/Info` | kinematic_node / graph_observer | Status + role |
| `/r_broadcast` | `swarm_control/RBroadcast` | reference | Leader’s reference broadcast |