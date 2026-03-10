# ============================================================
#  task_manager.py
#  ROS2 – Distributed Grid Fleet Coordination System
#  Responsible for: Generating tasks and assigning them to vehicles
# ============================================================

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import random
import threading
from grid_interfaces.srv import RequestTask

class TaskManagerNode(Node):

    def __init__(self):
        super().__init__('task_manager')

        # --- Internal task storage ---
        self.tasks = []
        # A list of dicts. Each dict represents one delivery task:

        self.completed_count = 0

        self.lock = threading.Lock()
 
        # --- Generate tasks at startup ---
        self.generate_tasks(count=12)

        # --- Callback group ---
        self.cb_group = MutuallyExclusiveCallbackGroup()

        # --- Service server ---
        self.task_service = self.create_service(
            RequestTask,             
            '/request_task',          
            self.handle_request_task,
            callback_group=self.cb_group
        )

        self.get_logger().info(
            f'Task Manager ready. {len(self.tasks)} tasks loaded.'
        )


    def generate_tasks(self, count: int):
        # Generates 'count' random delivery tasks and stores them
        # in self.tasks.

        generated = 0

        while generated < count:
            pickup_x  = random.randint(0, 7)
            pickup_y  = random.randint(0, 7)
            dropoff_x = random.randint(0, 7)
            dropoff_y = random.randint(0, 7)


            if (pickup_x == dropoff_x) and (pickup_y == dropoff_y):
                continue

            task = {
                'pickup_x':  pickup_x,
                'pickup_y':  pickup_y,
                'dropoff_x': dropoff_x,
                'dropoff_y': dropoff_y,
                'assigned':  False
            }

            self.tasks.append(task)
            generated += 1

        # Log all generated tasks so we can see them in the terminal
        self.get_logger().info('Generated tasks:')
        for i, t in enumerate(self.tasks):
            self.get_logger().info(
                f'  Task {i+1}: '
                f'Pickup ({t["pickup_x"]},{t["pickup_y"]}) -> '
                f'Dropoff ({t["dropoff_x"]},{t["dropoff_y"]})'
            )


    def handle_request_task(self, request, response):

        vehicle_id = request.vehicle_id
        self.get_logger().info(
            f'Task request received from: {vehicle_id}'
        )

        with self.lock:
            # Acquire the thread lock before touching self.tasks.
            # 'with' ensures the lock is always released, even if
            # an exception occurs inside the block.

            # Find the first task that has not been assigned yet
            available_task = None
            for task in self.tasks:
                if not task['assigned']:
                    available_task = task
                    break
            # We iterate in order so tasks are assigned sequentially.
            # This ensures fairness — vehicles can't skip tasks.

            if available_task is None:
                # No unassigned tasks remain.
                response.success   = False
                response.pickup_x  = 0
                response.pickup_y  = 0
                response.dropoff_x = 0
                response.dropoff_y = 0

                self.get_logger().info(
                    f'No tasks available for {vehicle_id}.'
                )
            else:
                available_task['assigned'] = True

                response.success   = True
                response.pickup_x  = available_task['pickup_x']
                response.pickup_y  = available_task['pickup_y']
                response.dropoff_x = available_task['dropoff_x']
                response.dropoff_y = available_task['dropoff_y']

                # Count how many tasks are still unassigned
                remaining = sum(
                    1 for t in self.tasks if not t['assigned']
                )

                self.get_logger().info(
                    f'Assigned to {vehicle_id}: '
                    f'Pickup ({response.pickup_x},{response.pickup_y}) -> '
                    f'Dropoff ({response.dropoff_x},{response.dropoff_y}) '
                    f'| Remaining tasks: {remaining}'
                )

        # Lock is released here automatically by the 'with' block.

        return response


    def mark_task_completed(self):
        # Called when a vehicle finishes a delivery.
        # Increments the global completed task counter and logs progress.

        with self.lock:
            self.completed_count += 1
            total    = len(self.tasks)
            assigned = sum(1 for t in self.tasks if t['assigned'])
            completed = self.completed_count

        self.get_logger().info(
            f'Task completed! Total completed: {completed}'
        )

        if assigned == total:
            # Every task has been assigned at least once.
            # All remaining work is in-progress or done.
            self.get_logger().info(
                'ALL TASKS ASSIGNED. '
                f'Completed so far: {completed}/{total}'
            )


def main(args=None):
    rclpy.init(args=args)

    node = TaskManagerNode()

    executor = MultiThreadedExecutor()

    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Task Manager shutting down...')

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()