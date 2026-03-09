# Distributed Grid Fleet Coordination System

## Project Overview

This project simulates a distributed coordination system for a fleet of three autonomous vehicles operating within a $8 \times 8$ grid world. The system focuses entirely on logical movement (numeric coordinates) and does not utilize sensors, cameras, or external simulators like Gazebo.

---

## System Architecture

The system consists of **6 mandatory nodes**:

### 1. Vehicle Nodes (3 instances)

Each vehicle manages its own internal state machine to handle task lifecycles:


* **States**: `IDLE`, `REQUEST_TASK`, `MOVING_TO_PICKUP`, `MOVING_TO_DROPOFF`, `WAITING`, and `FINISHED` .


* **Actions**: Publishes current position and state, and requests permission via services before entering any cell .



### 2. Task Manager Node

* Generates and stores a list of at least 10 delivery tasks.


* Ensures tasks are assigned uniquely through the `/request_task` service.


* Tracks the count of completed tasks.


### 3. Traffic Controller Node

* The system's safety engine that tracks occupied grid cells.


* Manages the `/request_move` service to approve or reject vehicle movements.


* Prevents collisions (two vehicles in one cell) and head-on position swaps.


* Handles **Deadlock Recovery**: Grants priority to one vehicle if a deadlock is detected.



### 4. Monitor Node

* Subscribes to all vehicle states to provide a global system dashboard.


* Prints active tasks, waiting vehicles, and total completions .


* Detects "stuck" vehicles if they remain in the same state for over 10 seconds.



---

## Communication Protocol

| Type | Name | Data Fields |
| --- | --- | --- |
| **Topic** | `/vehicle_position` | Current $(x, y)$ coordinates.|
| **Topic** | `/vehicle_state` | Current node status (e.g., `MOVING`).|
| **Service** | `/request_task` | Assigns pickup/drop-off locations.|
| **Service** | `/request_move` | Validates movement to target cell.|

---

## Execution Instructions

Per the project requirements, nodes must be started manually in separate terminals to demonstrate distributed operation.



1. **Launch Infrastructure Nodes**:


```bash
# Terminal 1
ros2 run grid_fleet task_manager

# Terminal 2
ros2 run grid_fleet traffic_controller

# Terminal 3
ros2 run grid_fleet monitor

```


2. **Launch the Fleet**:


```bash
# Terminal 4
ros2 run grid_fleet vehicle_node --ros-args -p vehicle_id:=vehicle1

# Terminal 5
ros2 run grid_fleet vehicle_node --ros-args -p vehicle_id:=vehicle2

# Terminal 6
ros2 run grid_fleet vehicle_node --ros-args -p vehicle_id:=vehicle3

```

---

## Core Objectives & Safety

* **Collision Prevention**: No two vehicles may occupy the same cell at any time.


* **Deadlock Resolution**: Prevent infinite waiting during position swaps (e.g., Vehicle 1 at $(2,3)$ wants $(2,4)$ while Vehicle 2 at $(2,4)$ wants $(2,3)$).


* **Fairness**: The Traffic Controller ensures no vehicle "starves" while others complete tasks.


* **Gradual Movement**: All logical steps include a delay to simulate real travel time.

---
