# swarm_control_msgs

A ROS2 package containing custom messages for `swarm_control` package.

## Package Overview

```
swarm_control_msgs/
├── msg/
│   ├── Info.msg
│   └── RBroadcast.msg
└── README.md
```

## Message Definitions

### **`Info.msg`**
```msg
string id
string role
builtin_interfaces/Time stamp
bool is_active
string status_msg
int32 component_id
```
> Published by `graph_observer` and each `kinematic_node`.  
> Tracks each robot’s identity, role, and component assignment.



### **`RBroadcast.msg`**
```msg
string id
builtin_interfaces/Time stamp
geometry_msgs/Point point
```
> Published by `reference.py`.  
> Contains the reference trajectory point r(t) broadcast by a leader.

## Topic Overview (per bot)

| Topic | Type | Publisher | Description |
|--------|------|------------|--------------|
| `/bot_i/info` | `swarm_control/Info` | kinematic_node / graph_observer | Status + role |
| `/r_broadcast` | `swarm_control/RBroadcast` | reference | Leader’s reference broadcast |