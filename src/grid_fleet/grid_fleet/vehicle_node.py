# ============================================================
#  vehicle_node.py
#  ROS2 – Distributed Grid Fleet Coordination System
#  Responsible for: Vehicle A / B / C (same file, 3 instances)
# ============================================================

# --- IMPORTS ------------------------------------------------
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import time
import random

# --- CUSTOM INTERFACE IMPORTS --------------------------------
from grid_interfaces.msg import VehiclePosition
from grid_interfaces.msg import VehicleState
from grid_interfaces.srv import RequestTask
from grid_interfaces.srv import RequestMove

# --- STATE CONSTANTS -----------------------------------------
IDLE              = 'IDLE'               # The vehicle has just started or just finished a task.
REQUEST_TASK      = 'REQUEST_TASK'
MOVING_TO_PICKUP  = 'MOVING_TO_PICKUP'
MOVING_TO_DROPOFF = 'MOVING_TO_DROPOFF'
WAITING           = 'WAITING'
FINISHED          = 'FINISHED'


# --- VEHICLE NODE CLASS --------------------------------------
class VehicleNode(Node):
    def __init__(self, vehicle_id: str):
        super().__init__(vehicle_id)

        self.declare_parameter('vehicle_id', vehicle_id)

        self.vehicle_id = self.get_parameter('vehicle_id') \
                              .get_parameter_value().string_value

        self.state = IDLE

        self.x = 0
        self.y = 0
        self.pickup_x  = None
        self.pickup_y  = None
        self.dropoff_x = None
        self.dropoff_y = None

        self.waiting_since = None

        # --- Callback group ---
        self.cb_group = ReentrantCallbackGroup()

        # --- Publishers ---
        self.position_pub = self.create_publisher(VehiclePosition, '/vehicle_position', 10)
        self.state_pub = self.create_publisher(VehicleState, '/vehicle_state', 10)

        # --- Service Clients ---
        self.task_client = self.create_client(RequestTask, '/request_task',callback_group=self.cb_group)
        self.move_client = self.create_client(RequestMove,'/request_move',callback_group=self.cb_group)

        # --- Wait for services to be available ---
        self.get_logger().info(f'[{self.vehicle_id}] Waiting for services...')

        self.task_client.wait_for_service(timeout_sec=10.0)
        self.move_client.wait_for_service(timeout_sec=10.0)

        self.get_logger().info(f'[{self.vehicle_id}] Services found. Starting state machine.')

        # --- Timer (state machine driver) ---
        self.timer = self.create_timer(0.5, self.state_machine_callback, callback_group=self.cb_group)


    def state_machine_callback(self):

        if self.state == IDLE:
            self.handle_idle()

        elif self.state == REQUEST_TASK:
            self.handle_request_task()

        elif self.state == MOVING_TO_PICKUP:
            self.handle_moving(
                target_x=self.pickup_x,
                target_y=self.pickup_y,
                next_state=MOVING_TO_DROPOFF
            )
            # When we reach pickup, transition to MOVING_TO_DROPOFF.

        elif self.state == MOVING_TO_DROPOFF:
            self.handle_moving(
                target_x=self.dropoff_x,
                target_y=self.dropoff_y,
                next_state=FINISHED
            )
            # When we reach dropoff, transition to FINISHED.

        elif self.state == WAITING:
            self.handle_waiting()

        elif self.state == FINISHED:
            self.handle_finished()


    def handle_idle(self):
        self.get_logger().info(f'[{self.vehicle_id}] State: IDLE')
        time.sleep(random.uniform(0.2, 1.0))
        self.set_state(REQUEST_TASK)

    def handle_request_task(self):
        self.get_logger().info(f'[{self.vehicle_id}] Requesting task from Task Manager...')

        # Build the service request object
        request = RequestTask.Request()
        request.vehicle_id = self.vehicle_id

        # Call the service asynchronously
        future = self.task_client.call_async(request)

        # wait for response without recursive spin
        while rclpy.ok() and not future.done():
            time.sleep(0.1)

        response = future.result()

        if response is None:
            self.get_logger().warn(f'[{self.vehicle_id}] Task request failed. Retrying...')
            return
            # Returning here means the timer will call this again in 0.5s.

        if response.success:
            # Task was successfully assigned to us.
            self.pickup_x  = response.pickup_x
            self.pickup_y  = response.pickup_y
            self.dropoff_x = response.dropoff_x
            self.dropoff_y = response.dropoff_y
            self.get_logger().info(
                f'[{self.vehicle_id}] Task received: '
                f'Pickup ({self.pickup_x},{self.pickup_y}) → '
                f'Dropoff ({self.dropoff_x},{self.dropoff_y})'
            )
            self.set_state(MOVING_TO_PICKUP)
        else:
            # No tasks left in the Task Manager's list.
            self.get_logger().info(
                f'[{self.vehicle_id}] No tasks available. Waiting...'
            )
            time.sleep(2.0)

    def handle_moving(self, target_x: int, target_y: int, next_state: str):
        # MOVING_TO_PICKUP or MOVING_TO_DROPOFF:
        # Compute one step toward the target, ask for permission,
        # then actually move if approved.

        if self.x == target_x and self.y == target_y:
            # Already at target — transition to next state.
            self.get_logger().info(f'[{self.vehicle_id}] Reached ({target_x},{target_y})')
            self.set_state(next_state)
            return

        # --- Compute next step (Manhattan movement) ---
        next_x, next_y = self.compute_next_step(target_x, target_y)

        # --- Request move permission ---
        self.get_logger().info(
            f'[{self.vehicle_id}] Requesting move to '
            f'({next_x},{next_y})...'
        )

        request = RequestMove.Request()
        request.vehicle_id = self.vehicle_id
        request.target_x   = next_x
        request.target_y   = next_y

        future = self.move_client.call_async(request)
        
        # wait for response without recursive spin
        while rclpy.ok() and not future.done():
            time.sleep(0.1)

        response = future.result()

        if response is None:
            self.get_logger().warn(
                f'[{self.vehicle_id}] Move request failed (no response).'
            )
            return

        if response.approved:
            # Permission granted — move to the new cell.
            self.x = next_x
            self.y = next_y
            self.publish_position()
            self.get_logger().info(f'[{self.vehicle_id}] Moved to ({self.x},{self.y})')
            time.sleep(1.0)

        else:
            # Permission denied — another vehicle is in the way.
            self.get_logger().warn(
                f'[{self.vehicle_id}] Move to ({next_x},{next_y}) '
                f'DENIED. Waiting...'
            )
            self.set_state(WAITING)
            self.waiting_since = time.time()
            # Record when we started waiting so Monitor can detect
            # if we've been stuck for more than 10 seconds.

    def handle_waiting(self):
        # WAITING: Our last move was denied. We retry.
        elapsed = time.time() - self.waiting_since
        # How long have we been waiting?

        if elapsed > 10.0:
            # Been waiting more than 10 seconds — something is wrong.
            # The Traffic Controller should have broken the deadlock,
            # but we log a warning here as a safety net.
            self.get_logger().warn(
                f'[{self.vehicle_id}] STUCK for {elapsed:.1f}s! '
                f'Deadlock may not have been resolved.'
            )

        # Retry by going back to the appropriate moving state
        # based on where we are in the task.
        if self.x == self.pickup_x and self.y == self.pickup_y:
            # Already at pickup, retry toward dropoff
            self.set_state(MOVING_TO_DROPOFF)
        else:
            # Still on the way to pickup
            self.set_state(MOVING_TO_PICKUP)

        # Add jitter to avoid all waiting vehicles retrying in sync
        time.sleep(random.uniform(0.3, 1.0))

    def handle_finished(self):
        # FINISHED: Delivery complete. Loop back to request a new task.
        self.get_logger().info(
            f'[{self.vehicle_id}] Task COMPLETE. '
            f'Delivered to ({self.dropoff_x},{self.dropoff_y}).'
        )
        # Clear task data
        self.pickup_x  = None
        self.pickup_y  = None
        self.dropoff_x = None
        self.dropoff_y = None

        time.sleep(0.5)
        self.set_state(IDLE)


    def compute_next_step(self, target_x: int, target_y: int) -> tuple:
        # Computes the next single-cell step toward (target_x, target_y)
        # using a simple greedy Manhattan strategy.
        # Priority: move in x direction first, then y.
        # This ensures predictable, straight-line paths.

        next_x = self.x
        next_y = self.y

        if self.x < target_x:
            next_x = self.x + 1   # move right
        elif self.x > target_x:
            next_x = self.x - 1   # move left
        elif self.y < target_y:
            next_y = self.y + 1   # move down
        elif self.y > target_y:
            next_y = self.y - 1   # move up

        # Grid boundary clamp (safety check)
        next_x = max(0, min(7, next_x))
        next_y = max(0, min(7, next_y))
        # Ensures we never go outside the 8x8 grid (0–7 range).

        return next_x, next_y

    def set_state(self, new_state: str):
        self.state = new_state
        self.publish_state()

    def publish_position(self):
        msg = VehiclePosition()
        msg.vehicle_id = self.vehicle_id
        msg.x = self.x
        msg.y = self.y
        self.position_pub.publish(msg)

    def publish_state(self):
        msg = VehicleState()
        msg.vehicle_id = self.vehicle_id
        msg.state = self.state
        self.state_pub.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    # --- Read vehicle_id from ROS2 parameter override ---
    # We create a temporary minimal node to read the parameter,
    # then create the real VehicleNode with the correct ID.
    import sys
    vehicle_id = 'vehicle1'   # default fallback
    for arg in sys.argv:
        if arg.startswith('vehicle_id:='):
            vehicle_id = arg.split(':=')[1]

    node = VehicleNode(vehicle_id)

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()

    except KeyboardInterrupt:
        node.get_logger().info(
            f'[{vehicle_id}] Shutting down...'
        )

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
    