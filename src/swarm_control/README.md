# 🐝 swarm_control

A ROS 2 package for **decentralized swarm coordination and control** using a **finite-time kinematic model** and **dynamic leader election** based on graph connectivity.

This package simulates a multi-robot system (e.g., TurtleBot3s in Gazebo) where each agent interacts only with its neighbors within a vision radius.  
The system collectively tracks a reference trajectory in a **distributed** and **leader-adaptive** manner.



## 📁 Package Overview

```
swarm_control/
├── msg/
│   ├── Info.msg
│   └── RBroadcast.msg
├── src/
│   ├── kinematic_node.py
│   ├── reference.py
│   ├── graph_observer.py
│   └── graph_utils.py
├── launch/
│   ├── swarm_launch.py
│   └── gazebo_world.launch.py
└── README.md
```



## 🧩 Nodes Summary

### 1. `graph_observer.py`
**Role:** Central node that monitors the swarm graph in real time.

- **Subscribes:** `/bot_i/odom` for all robots  
- **Builds:** adjacency matrix based on Euclidean distance ≤ `delta_radius`  
- **Finds:** connected components  
- **Elects:** leader = agent with highest degree in each component  
- **Publishes:** `/bot_i/info` (`Info.msg`) with `role`, `is_active`, and `component_id`



### 2. `reference.py`
**Role:** Global reference generator and leader broadcaster.

- **Publishes:**  
  - `/reference` (`geometry_msgs/PointStamped`) → trajectory r(t) = [t, sin(t)]  
  - `/r_broadcast` (`RBroadcast.msg`) → leaders’ broadcast of r(t)
- **Subscribes:** `/bot_i/info` for leader identification  
- **Behavior:** Leaders detected by `graph_observer` are used to forward r(t) to their components.



### 3. `kinematic_node.py`
**Role:** Local agent control node implementing the finite-time kinematic model.

- **Subscribes:**
  - `/bot_i/odom` (self)
  - `/bot_j/odom` (others) → determine neighbors  
  - `/bot_j/delta` (others) → get neighbor Δ values  
  - `/r_broadcast` → get current reference r(t)
- **Publishes:**
  - `/bot_i/delta` (`geometry_msgs/PointStamped`) → agent’s Δ state  
  - `/bot_i/info` (`Info.msg`) → status broadcast
- **Computation:**
  Δ̇ᵢ = -Δᵢ - (M / |𝒩ᵢ|+1) Σⱼ∈𝒩ᵢ [(Δᵢ-Δⱼ)/(‖Δᵢ-Δⱼ‖^ν+ε)]
  and Euler integration at Tₛ.



## 🧾 Message Definitions

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



## ⚙️ Parameters

| Parameter | Node | Description | Default |
|------------|------|--------------|----------|
| `num_bots` | all | Number of robots in the swarm | `3` |
| `bot_id` | kinematic_node | Robot namespace ID (e.g. `"bot1"`) | — |
| `role` | kinematic_node | `"leader"` or `"agent"` | `"agent"` |
| `delta_radius` | all | Vision/communication radius [m] | `3.0` |



## 🚀 Running the Simulation

### 1️⃣ Build
```bash
cd ~/ros2_ws
colcon build --packages-select swarm_control
source install/setup.bash
```

### 2️⃣ Launch Swarm Control Stack
```bash
ros2 launch swarm_control swarm_launch.py num_bots:=4
```

This will:
- Spawn N robots in Gazebo  
- Launch one `kinematic_node.py` per bot  
- Start `graph_observer.py`  
- Start `reference.py`



## 📡 Topic Overview (per bot)

| Topic | Type | Publisher | Description |
|--------|------|------------|--------------|
| `/bot_i/odom` | `nav_msgs/Odometry` | Gazebo | Ground-truth position |
| `/bot_i/delta` | `geometry_msgs/PointStamped` | kinematic_node | Local Δ(t) state |
| `/bot_i/info` | `swarm_control/Info` | kinematic_node / graph_observer | Status + role |
| `/r_broadcast` | `swarm_control/RBroadcast` | reference | Leader’s reference broadcast |
| `/reference` | `geometry_msgs/PointStamped` | reference | True reference trajectory |



## 🧠 Algorithmic Flow

1. **Graph Construction:** `graph_observer` builds a time-varying adjacency graph based on inter-robot distances.  
2. **Leader Election:** The robot with the **highest degree** in each connected component becomes leader.  
3. **Reference Broadcast:** Leaders publish the reference r(t) to `/r_broadcast`.  
4. **Decentralized Control:** Each agent computes Δᵢ = zᵢ − r(t) using neighbor states and performs the finite-time update.  
5. **Dynamic Reconfiguration:** When components merge/split, `graph_observer` redefines leaders and publishes updated roles.



## 🧩 Extending the Package

- Add a **Model Predictive Control (MPC)** layer for velocity control using `/bot_i/delta` as the input reference.  
- Include a **visualization node** publishing `visualization_msgs/Marker` lines for graph edges in RViz.  
- Implement health metrics in `Info.msg` for real-world diagnostics.  
- Integrate `rclpy.lifecycle` nodes for fault-tolerant reconfiguration.



## 📖 Dependencies

- **ROS 2 Jazzy**
- `geometry_msgs`
- `nav_msgs`
- `rclpy`
- `numpy`



## 🧑‍💻 Author & Maintainers

Developed by **Yash Purswani** and **Trisha Wadhwani**
as part of a decentralized swarm control project for **ME5253: Network Dynamics and Controls** at **IIT Madras**.