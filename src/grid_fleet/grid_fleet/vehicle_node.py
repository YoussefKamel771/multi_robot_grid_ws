import rclpy
from rclpy.node import Node
from grid_interfaces.msg import VehiclePosition, VehicleState
from grid_interfaces.srv import RequestTask, RequestMove
import random

class VehicleNode(Node):
    def __init__(self):
        super().__init__('vehicle_node')

        self.declare_parameter("vehicle_id", "vehicle1")
        self.vehicle_id = self.get_parameter("vehicle_id").value
        self.x = random.randint(0, 7)
        self.y = random.randint(0, 7)
        self.state = "IDLE"

        self.task_msg = RequestTask.Request()
        self.task_msg.vehicle_id = self.vehicle_id
        self.move_msg = RequestMove.Request()
        self.move_msg.vehicle_id = self.vehicle_id
        

        self.position_pub = self.create_publisher(VehiclePosition, '/vehicle_position', 10)
        self.state_pub = self.create_publisher(VehicleState, '/vehicle_state', 10)

        self.task_client = self.create_client(RequestTask, '/request_task')
        self.move_client = self.create_client(RequestMove, '/request_move')


        self.get_logger().info(f"Started Vehicle with id:{self.vehicle_id}")

        self.publish_position()
        self.publish_state()
        while not self.task_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Waiting for task service...")
        while not self.move_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for move service...")
        
        self.get_logger().info("All services available, starting main loop")
        self.timer = self.create_timer(1.0, self.loop)

    def publish_position(self):
        msg = VehiclePosition()
        msg.vehicle_id = self.vehicle_id
        msg.x = self.x
        msg.y = self.y
        self.position_pub.publish(msg)
        self.get_logger().debug(f"[{self.vehicle_id}] Published position: ({self.x}, {self.y})")

    def publish_state(self):
        msg = VehicleState()
        msg.vehicle_id = self.vehicle_id
        msg.state = self.state
        self.state_pub.publish(msg)
        self.get_logger().debug(f"[{self.vehicle_id}] Published state: {self.state}")


    def loop(self):
        self.publish_position()
        self.publish_state()

        if self.state == "IDLE":
            self.get_logger().debug(f"[{self.vehicle_id}] Transition: IDLE -> REQUEST_TASK")
            self.state = "REQUEST_TASK"
        
        elif self.state == "REQUEST_TASK":
            self.get_logger().info(f"[{self.vehicle_id}] Requesting new task")
            self.request_task()

        elif self.state == "MOVING_TO_PICKUP":
            if self.x != self.pickup_x or self.y != self.pickup_y:
                self.request_pickup_move()
            else:
                self.get_logger().info(f"[{self.vehicle_id}] Pickup reached at ({self.x}, {self.y})")
                self.state = "MOVING_TO_DROPOFF"
                return

        elif self.state == "MOVING_TO_DROPOFF":
            if self.x != self.dropoff_x or self.y != self.dropoff_y:
                self.request_dropoff_move()
            else:
                self.get_logger().info(f"[{self.vehicle_id}] Dropoff reached at ({self.x}, {self.y})")
                self.state = "FINISHED"
                return
        elif self.state == "FINISHED":
            self.request_task()
        
        elif self.state == "WAITING":
            self.get_logger().warn(f"[{self.vehicle_id}] Waiting: retrying last move")
            future = self.move_client.call_async(self.move_msg)
            rclpy.spin_until_future_complete(self, future)
            result = future.result()
            if result and result.approved:
                self.x = self.next_x
                self.y = self.next_y
                # Resume previous goal
                self.state = self.pre_waiting_state
            
    def request_task(self):
        try:
            future = self.task_client.call_async(self.task_msg)
            rclpy.spin_until_future_complete(self, future)
        except Exception as e:
            self.get_logger().error(f"Task service call failed for vehicle '{self.vehicle_id}': {repr(e)}")
            return

        if future.result() is None:
            self.get_logger().warn(f"Task service returned no result for vehicle '{self.vehicle_id}'")
            return

        result = future.result()
        if result.success:
            self.task_accepted(result)
        else:
            self.get_logger().info(f"No task assigned to vehicle '{self.vehicle_id}' (success flag false)")

    def task_accepted(self, result):
        self.pickup_x = result.pickup_x
        self.pickup_y = result.pickup_y
        self.dropoff_x = result.dropoff_x
        self.dropoff_y = result.dropoff_y
        self.get_logger().info(
            f"[{self.vehicle_id}] New task: "
            f"Pickup ({result.pickup_x}, {result.pickup_y}) -> "
            f"Dropoff ({result.dropoff_x}, {result.dropoff_y})"
        )
        self.state = "MOVING_TO_PICKUP"
 
    def request_pickup_move(self):
        self.compute_next_pickup()
        future = self.move_client.call_async(self.move_msg)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if result is None:
            self.get_logger().warn(f"[{self.vehicle_id}] Move service returned no result (pickup)")
            return
        if result.approved:
            self.x = self.next_x
            self.y = self.next_y
            self.get_logger().info(f"[{self.vehicle_id}] Move approved (pickup), new pos=({self.x}, {self.y})")
        else:
            self.get_logger().warn(f"[{self.vehicle_id}] Move to pickup not approved, WAITING")
            self.pre_waiting_state = self.state
            self.state = "WAITING"

    def request_dropoff_move(self):
        self.compute_next_dropoff()
        future = self.move_client.call_async(self.move_msg)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if result is None:
            self.get_logger().warn(f"[{self.vehicle_id}] Move service returned no result (dropoff)")
            return
        if result.approved:
            self.x = self.next_x
            self.y = self.next_y
            self.get_logger().info(f"[{self.vehicle_id}] Move approved (pickup), new pos=({self.x}, {self.y})")
        else:
            self.get_logger().warn(f"[{self.vehicle_id}] Move to dropoff not approved, WAITING")
            self.pre_waiting_state = self.state
            self.state = "WAITING"

    def compute_next_pickup(self):
        self.next_x = self.x
        self.next_y = self.y
        if self.pickup_x > self.x:
            self.next_x += 1
        elif self.pickup_x < self.x:
            self.next_x -= 1
        elif self.pickup_y > self.y:
            self.next_y += 1
        elif self.pickup_y < self.y:
            self.next_y -= 1
        self.move_msg.target_x = self.next_x
        self.move_msg.target_y = self.next_y
        self.get_logger().debug(f"[{self.vehicle_id}] compute_next_pickup: ({self.x}, {self.y}) -> ({self.next_x}, {self.next_y})")

    def compute_next_dropoff(self):
        self.next_x = self.x
        self.next_y = self.y
        if self.dropoff_x > self.x:
            self.next_x += 1
        elif self.dropoff_x < self.x:
            self.next_x -= 1
        elif self.dropoff_y > self.y:
            self.next_y += 1
        elif self.dropoff_y < self.y:
            self.next_y -= 1
        self.move_msg.target_x = self.next_x
        self.move_msg.target_y = self.next_y
        self.get_logger().debug(f"[{self.vehicle_id}] compute_next_dropoff: ({self.x}, {self.y}) -> ({self.next_x}, {self.next_y})")

def main(args=None):
    rclpy.init(args=args)
    node = VehicleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()