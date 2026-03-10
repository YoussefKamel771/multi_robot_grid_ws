#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from grid_interfaces.msg import VehiclePosition, VehicleState
from grid_interfaces.srv import RequestMove, RequestTask


class monitorNode(Node):
    def __init__(self):
        super().__init__("monitor_node")

        self.vehicles = {}
        self.states = {}

        self.position_sub = self.create_subscription(
            VehiclePosition,
            "/vehicle_position", # To be Modified
            self.vehicle_pos_callback,
            10
        )

        self.state_sub = self.create_subscription(
            VehicleState,
            '/vehicle_state',
            self.vehicle_state_callback, # To be Modified
            10
        )

        self.timer = self.create_timer(1, self.processor)

        self.get_logger().info("Monitor Started")

    def processor(self):
        if not self.vehicles:
            return
        for vehicle_id, data in self.vehicles.items():
            state = data.get("state", "-")
            x = data.get("x", "-")
            y = data.get("y", "-")
            self.get_logger().info(f"Vehicle id: {vehicle_id}:\n \
                                    State: {state}, Current position: ({x}, {y})")

            if vehicle_id not in self.states:
                self.states[vehicle_id] = {"state": state, "stuck_counter": 0}
                continue

            self.states[vehicle_id]["state"] = state
            if self.states[vehicle_id]["state"] == "WAITING":
                self.states[vehicle_id]["stuck_counter"] += 1
            else:
                self.states[vehicle_id]["stuck_counter"] = 0

            if self.states[vehicle_id]["stuck_counter"] >= 10:
                self.get_logger().warn(f"Vehicle: {vehicle_id} might be stuck")

    def vehicle_pos_callback(self, msg:VehiclePosition):
        vehicle_id = msg.vehicle_id

        if vehicle_id not in self.vehicles:
            self.vehicles[vehicle_id] = {}

        self.vehicles[vehicle_id]["x"] = msg.x
        self.vehicles[vehicle_id]["y"] = msg.y

    def vehicle_state_callback(self, msg:VehicleState):
        vehicle_id = msg.vehicle_id

        if vehicle_id not in self.vehicles:
            self.vehicles[vehicle_id] = {}

        self.vehicles[vehicle_id]["state"] = msg.state

def main(args=None):
    rclpy.init(args=args)
    myNode = monitorNode()
    rclpy.spin(myNode)
    rclpy.shutdown()

if __name__ == "__main__":
    main()