# Distributed Grid Fleet Coordination System — ROS2 Plan

## System Overview


```
┌──────────────┐     /request_task (service)      ┌─────────────────┐
│  Vehicle A   │────────────────────────────────▶ │                 │
│  Vehicle B   │────────────────────────────────▶ │  Task Manager   │
│  Vehicle C   │────────────────────────────────▶ │                 │
└──────┬───────┘                                  └─────────────────┘
       │
       │  /vehicle_position (topic)
       │  /vehicle_state    (topic)
       │                                          ┌─────────────────┐
       ├─────────────────────────────────────────▶│    Traffic      │
       │  /request_move (service)                 │   Controller   │
       │◀────────────────────────────────────────▶│                 │
       │                                          └─────────────────┘
       │  /vehicle_state (topic)
       │                                          ┌─────────────────┐
       └─────────────────────────────────────────▶│    Monitor      │
                                                  │     Node        │
                                                  └─────────────────┘
```

---

## Step 1 — Define Custom Messages & Services

This is the foundation. Everything else depends on these interfaces.

### Topics (Messages)
You need two custom message types:

**VehiclePosition.msg**
- `string vehicle_id` — which vehicle (vehicle1, vehicle2, vehicle3)
- `int32 x` — current x on the 8x8 grid
- `int32 y` — current y on the 8x8 grid

**VehicleState.msg**
- `string vehicle_id`
- `string state` — one of: IDLE, REQUEST_TASK, MOVING_TO_PICKUP, MOVING_TO_DROPOFF, WAITING, FINISHED

### Services
You need two custom service types:

**RequestTask.srv**
- Request: `string vehicle_id`
- Response: `bool success`, `int32 pickup_x`, `int32 pickup_y`, `int32 dropoff_x`, `int32 dropoff_y`

**RequestMove.srv**
- Request: `string vehicle_id`, `int32 target_x`, `int32 target_y`
- Response: `bool approved`


---

## Step 2 — Package Structure

You'll create **one ROS2 package** (or two: one for interfaces, one for nodes).

```
grid_fleet/
├── grid_fleet/
│   ├── vehicle_node.py       ← runs 3 times with different names
│   ├── task_manager.py
│   ├── traffic_controller.py
│   └── monitor_node.py
│ 
├── package.xml
└── setup.py

grid_interfaces/
├── msg/
│   ├── VehiclePosition.msg
│   └── VehicleState.msg
├── srv/
│   ├── RequestTask.srv
│   └── RequestMove.srv
├── CMakeLists.txt
└── package.xml
```


---

## Step 3 — Node-by-Node Plan

### 🚗 Vehicle Node (Student 1, 2, 3 — same file, 3 instances)

This is the most complex node. It runs as a **state machine**.

**What it does internally:**

1. **Starts in IDLE** — waits briefly then transitions to REQUEST_TASK
2. **REQUEST_TASK** — calls `/request_task` service, sends its ID, waits for a task (pickup x,y and dropoff x,y)
3. **MOVING_TO_PICKUP** — computes the next step toward pickup using simple Manhattan movement (move one cell at a time: prioritize x first, then y). Before each move, calls `/request_move`
4. **WAITING** — if `/request_move` returns `approved=false`, it waits and retries after a short delay
5. **MOVING_TO_DROPOFF** — same logic as MOVING_TO_PICKUP but heading to dropoff
6. **FINISHED** — reports completion, loops back to REQUEST_TASK

**Publishing:**
- Publishes to `/vehicle_position` every time it moves
- Publishes to `/vehicle_state` every time its state changes

**How to run 3 instances:**
In ROS2, you pass parameters or use remapping to give each instance a unique ID:
```
ros2 run grid_fleet vehicle_node --ros-args -p vehicle_id:=vehicle1
```

---

### 📋 Task Manager Node (Student 4)

**What it does:**

1. At startup, **generates 10+ tasks** — random (x,y) pickup and dropoff pairs within the 8x8 grid (0–7 range). Store them in a list.
2. Maintains an internal counter of how many have been assigned and completed.
3. **Provides `/request_task` service** — when a vehicle calls it, pops the next unassigned task from the list and returns it. Marks it as assigned.
4. Tracks completed tasks (vehicles could notify via a separate topic or service — your choice).
5. Handles **simultaneous requests** — because ROS2 services are handled sequentially by a single-threaded executor by default, two simultaneous requests are automatically queued. However, using a **MultiThreadedExecutor** with a **ReentrantCallbackGroup** is the ROS2 way to handle concurrent service calls safely.

---

### 🚦 Traffic Controller Node (Student 5 — Part 1)

This is the most critical node.

**What it does:**

1. **Subscribes to `/vehicle_position`** — maintains a dictionary: `{(x,y): vehicle_id}` of all currently occupied cells.
2. **Provides `/request_move` service** — when a vehicle requests to enter cell (x,y):
   - Check if (x,y) is already occupied → **reject**
   - Check for **direct swap deadlock**: if Vehicle A is at position P1 requesting P2, and Vehicle B is at P2 requesting P1 → one must be rejected and given priority
   - If clear → **approve** and update the occupied cells map
3. **Deadlock detection:** uses a timer to check if any vehicle has been WAITING longer than a threshold. If deadlock detected, it grants priority to one vehicle (e.g., the one with lower ID or the one that's been waiting longer).
4. **When a vehicle moves**, the old cell must be **freed** — the Traffic Controller updates this when it receives the new position.

**Deadlock Example Handling:**
- Vehicle 1 at (2,3) wants (2,4) — approved, Vehicle 2 at (2,4) wants (2,3) — REJECTED (swap detected)
- Vehicle 2 must try an alternative path (go around)

---

### 📊 Monitor Node (Student 5 — Part 2)

**What it does:**

1. **Subscribes to `/vehicle_state`** and `/vehicle_position` for all vehicles
2. Maintains a record of each vehicle's current state and a timestamp of when it entered that state
3. **Prints a status summary** periodically (e.g., every 2 seconds):
   - Which vehicles are active and what task they're on
   - Which vehicles are WAITING and for how long
   - How many tasks are completed
4. **Stuck detection:** if a vehicle has been in WAITING or any non-moving state for more than **10 seconds**, it prints a warning

---

## Step 4 — Execution Plan (ROS2 Way)

Since the PDF says "no launch files" — you open **6 terminals** and run:

```bash
# Terminal 1
ros2 run grid_fleet task_manager

# Terminal 2
ros2 run grid_fleet traffic_controller

# Terminal 3
ros2 run grid_fleet monitor

# Terminal 4
ros2 run grid_fleet vehicle_node --ros-args -p vehicle_id:=vehicle1

# Terminal 5
ros2 run grid_fleet vehicle_node --ros-args -p vehicle_id:=vehicle2

# Terminal 6
ros2 run grid_fleet vehicle_node --ros-args -p vehicle_id:=vehicle3
```


---

## Summary of Development Order

1. **First** — set up the package and define all messages/services
2. **Second** — build the Task Manager (simplest, no dependencies)
3. **Third** — build the Traffic Controller (core logic)
4. **Fourth** — build the Vehicle Node (depends on both services)
5. **Fifth** — build the Monitor (just subscribes, no logic)
6. **Last** — test all together, verify deadlock handling works

