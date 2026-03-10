# ============================================================
#  traffic_controller.py
#  ROS2 – Distributed Grid Fleet Coordination System
#  Responsible for: Tracking positions, approving/rejecting moves,
#                   preventing collisions, and breaking deadlocks
# ============================================================


import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import threading
import time
from grid_interfaces.msg import VehiclePosition
from grid_interfaces.srv import RequestMove

class TrafficControllerNode(Node):

    def __init__(self):
        super().__init__('traffic_controller')

        # --- Shared data structures ---

        self.occupied_cells = {}
        # A dictionary mapping (x, y) tuples to vehicle_id strings.

        self.vehicle_positions = {}
        # A dictionary mapping vehicle_id to its last known (x, y).
 
        self.pending_requests = {}
        # A dictionary mapping vehicle_id to the cell it is
        # currently trying to move into.
        # Example: { 'vehicle1': (2,4), 'vehicle2': (2,3) }
        #
        # This snapshot is used to detect deadlocks:
        # if vehicle1 wants vehicle2's cell AND vehicle2 wants
        # vehicle1's cell, we have a direct swap deadlock.

        self.waiting_since = {}
        # A dictionary mapping vehicle_id to the timestamp (float)
        # of when it was first denied a move in the current wait cycle.
        # Example: { 'vehicle2': 1712345678.32 }
        #
        # Used to decide who gets priority when breaking a deadlock:
        # the vehicle that has been waiting LONGER gets to move first.

        self.priority_vehicle = None
        # When a deadlock is detected, one vehicle is chosen as the
        # "priority" vehicle and its next move request is force-approved.
        # This field stores that vehicle's ID until it successfully moves.

        # --- Thread lock ---
        self.lock = threading.Lock()
        # Protects ALL the dictionaries above.
        # The /request_move service and /vehicle_position subscription
        # callbacks can run on different threads simultaneously.
        # Without this lock, both could modify occupied_cells at the
        # same moment, causing silent data corruption.

        # --- Callback group ---
        self.cb_group = ReentrantCallbackGroup()
        # We use ReentrantCallbackGroup so that the move service
        # callback and the position subscription callback can
        # both run at the same time on different threads.
        # The lock above is what keeps them safe — not serialization.

        # --- Subscriber ---
        self.position_sub = self.create_subscription(
            VehiclePosition,          
            '/vehicle_position',      
            self.handle_position_update,  
            10,                       
            callback_group=self.cb_group
        )

        # --- Service server ---
        self.move_service = self.create_service(
            RequestMove,
            '/request_move',
            self.handle_request_move,
            callback_group=self.cb_group
        )

        # --- Deadlock detection timer ---
        self.deadlock_timer = self.create_timer(
            1.0,                          
            self.check_for_deadlocks,
            callback_group=self.cb_group
        )

        self.get_logger().info('Traffic Controller ready.')


    def handle_position_update(self, msg):
        # Called every time a vehicle publishes a new position.
        # msg.vehicle_id : which vehicle moved
        # msg.x, msg.y   : its NEW position on the grid

        vehicle_id = msg.vehicle_id
        new_pos    = (msg.x, msg.y)

        with self.lock:

            # Step 1: Free the old cell if we knew where this vehicle was
            if vehicle_id in self.vehicle_positions:
                old_pos = self.vehicle_positions[vehicle_id]

                if old_pos in self.occupied_cells:
                    if self.occupied_cells[old_pos] == vehicle_id:
                        # Only remove if THIS vehicle was recorded there.
                        # Safety check: prevents removing another vehicle's
                        # cell if positions overlap during an edge case.
                        del self.occupied_cells[old_pos]

            # Step 2: Record the new position
            self.vehicle_positions[vehicle_id] = new_pos
            self.occupied_cells[new_pos]       = vehicle_id
            # Mark the new cell as taken by this vehicle.

            # Step 3: Clear this vehicle's pending request and wait timer
            # because it successfully moved — wait cycle is over.
            if vehicle_id in self.pending_requests:
                del self.pending_requests[vehicle_id]

            if vehicle_id in self.waiting_since:
                del self.waiting_since[vehicle_id]

            # Step 4: If this was the priority vehicle, clear priority
            # since it has now moved out of the deadlock.
            if self.priority_vehicle == vehicle_id:
                self.priority_vehicle = None
                self.get_logger().info(
                    f'{vehicle_id} moved as priority vehicle. '
                    f'Deadlock resolved.'
                )

        self.get_logger().debug(
            f'Position update: {vehicle_id} -> {new_pos}'
        )


    def handle_request_move(self, request, response):
        self.get_logger().info(f"received request {request}")
        vehicle_id = request.vehicle_id
        target     = (request.target_x, request.target_y)

        with self.lock:

            # --- Record this pending request ---
            self.pending_requests[vehicle_id] = target

            # --- Check 1: Is the target cell occupied? ---
            if target in self.occupied_cells:
                occupant = self.occupied_cells[target]

                if occupant == vehicle_id:
                    # The vehicle is already in this cell (stale data).
                    # Approve so it can resync and move on.
                    response.approved = True
                    self.get_logger().warn(
                        f'{vehicle_id} requested its own cell {target}. '
                        f'Auto-approved to resync.'
                    )
                    return response

                # Cell is occupied by a different vehicle.

                # --- Check 2: Direct swap deadlock ---
                # A direct swap is: vehicle A wants B's cell AND
                # vehicle B currently wants A's cell.
                # If allowed, both would move into each other simultaneously
                # which is a collision disguised as movement.
                is_swap = self._is_direct_swap(vehicle_id, occupant, target)

                if is_swap:
                    # Force-grant priority to the vehicle that has been
                    # waiting longer, or the requesting vehicle if neither
                    # has waited (fresh deadlock).
                    priority = self._resolve_swap_priority(
                        vehicle_id, occupant
                    )

                    if priority == vehicle_id:
                        # This vehicle gets to move through the swap.
                        # The other vehicle will be rejected on its next
                        # attempt and must find an alternative path.
                        response.approved  = True
                        self.priority_vehicle = vehicle_id
                        self.get_logger().warn(
                            f'SWAP DEADLOCK detected between '
                            f'{vehicle_id} and {occupant}. '
                            f'Granting priority to {vehicle_id}.'
                        )
                        # Update occupied_cells preemptively so the
                        # next check from occupant will see this cell taken.
                        # (The position update from vehicle_id will confirm it.)
                    else:
                        # The other vehicle has priority — this one waits.
                        response.approved = False
                        self._record_waiting(vehicle_id)
                        self.get_logger().warn(
                            f'SWAP DEADLOCK: {vehicle_id} must wait. '
                            f'{occupant} has priority.'
                        )
                else:
                    # Not a swap — just a normal occupied cell.
                    # Reject and let the vehicle wait and retry.
                    response.approved = False
                    self._record_waiting(vehicle_id)
                    self.get_logger().info(
                        f'{vehicle_id} -> {target} DENIED. '
                        f'Cell occupied by {occupant}.'
                    )

            else:
                # --- Cell is free ---

                # --- Check 3: Grid boundary ---
                if not self._is_valid_cell(target):
                    response.approved = False
                    self.get_logger().warn(
                        f'{vehicle_id} requested out-of-bounds '
                        f'cell {target}. DENIED.'
                    )
                    return response

                # All clear — approve the move.
                response.approved = True

                # Pre-mark the cell as occupied immediately.
                # This closes the race window between approval and
                # the actual position-update message arriving.
                # Without this, two vehicles could both be approved
                # for the same cell if their requests arrive in the
                # same millisecond.
                self.occupied_cells[target] = vehicle_id

                self.get_logger().info(
                    f'{vehicle_id} -> {target} APPROVED.'
                )

        return response


    def check_for_deadlocks(self):
        # Runs in the background every second.
        # Looks for vehicles stuck waiting longer than the threshold
        # and resolves any circular wait patterns beyond simple swaps.

        DEADLOCK_THRESHOLD = 5.0
        # If a vehicle has been waiting more than 5 seconds,
        # we consider it potentially deadlocked and intervene.
        # The Monitor Node separately flags vehicles stuck > 10 seconds.

        now = time.time()

        with self.lock:

            # Find all vehicles that have been waiting too long
            stuck_vehicles = [
                vid for vid, since in self.waiting_since.items()
                if (now - since) > DEADLOCK_THRESHOLD
            ]

            if not stuck_vehicles:
                return
            # No stuck vehicles — nothing to do this cycle.

            self.get_logger().warn(
                f'Deadlock check: stuck vehicles = {stuck_vehicles}'
            )

            # --- Circular deadlock detection ---
            # Beyond the direct swap (A wants B, B wants A),
            # we might have a chain: A wants B's cell, B wants C's cell,
            # C wants A's cell. This is a 3-way circular wait.
            #
            # Strategy: pick the vehicle that has been waiting the LONGEST
            # and grant it priority. It gets an unconditional pass on its
            # next /request_move call.

            if self.priority_vehicle is not None:
                # A priority was already granted but the vehicle hasn't
                # moved yet. Don't override — give it more time.
                return

            # Sort stuck vehicles by how long they have been waiting.
            # The one waiting longest gets priority.
            stuck_vehicles.sort(
                key=lambda vid: self.waiting_since.get(vid, now)
            )
            # .get(vid, now) falls back to current time if somehow
            # the key disappeared between the list build and sort.

            chosen = stuck_vehicles[0]
            # The vehicle with the earliest (smallest) timestamp
            # has been waiting the longest.

            self.priority_vehicle = chosen
            self.get_logger().warn(
                f'DEADLOCK RESOLVED: granting priority to {chosen}. '
                f'It has been waiting '
                f'{now - self.waiting_since[chosen]:.1f}s.'
            )


    def _is_direct_swap(self, requester: str,
                        occupant: str, target: tuple) -> bool:
        # Returns True if a direct position swap is detected.
        #
        # A swap exists when:
        #   - 'requester' is at position P1 and wants to move to P2
        #   - 'occupant'  is at position P2 (which IS the target)
        #     and has a pending request to move to P1
        #
        # Example:
        #   vehicle1 at (2,3) wants (2,4)  [target = (2,4)]
        #   vehicle2 at (2,4) wants (2,3)
        #   → direct swap detected

        # Where is the requester right now?
        requester_pos = self.vehicle_positions.get(requester)
        if requester_pos is None:
            return False
        # If we don't know the requester's position yet (just started),
        # we can't determine a swap — treat as non-swap.

        # Does the occupant have a pending request to go where
        # the requester currently is?
        occupant_wants = self.pending_requests.get(occupant)

        return occupant_wants == requester_pos
        # True only if occupant is actively trying to move INTO
        # the requester's current cell — the classic swap scenario.

    def _resolve_swap_priority(self, v1: str, v2: str) -> str:
        # Given two vehicles in a swap deadlock, returns the ID of
        # whichever has been waiting longer and should move first.
        # If neither has a recorded wait time (brand-new deadlock),
        # defaults to v1 (the one who made the current request).

        now = time.time()
        v1_wait = self.waiting_since.get(v1, now)
        v2_wait = self.waiting_since.get(v2, now)

        if v1_wait <= v2_wait:
            return v1
            # v1 started waiting earlier (smaller timestamp = longer wait).
        else:
            return v2

    def _record_waiting(self, vehicle_id: str):
        # Records the start time of a vehicle's wait, but only if
        # this is the BEGINNING of its wait cycle.
        # If it is already recorded as waiting, we do NOT overwrite
        # the timestamp — we want to preserve how long it has been
        # waiting in total, not just the most recent rejection.

        if vehicle_id not in self.waiting_since:
            self.waiting_since[vehicle_id] = time.time()

    def _is_valid_cell(self, pos: tuple) -> bool:
        # Returns True if the (x, y) position is within the 8x8 grid.
        x, y = pos
        return 0 <= x <= 7 and 0 <= y <= 7


def main(args=None):
    rclpy.init(args=args)

    node = TrafficControllerNode()

    executor = MultiThreadedExecutor()

    executor.add_node(node)

    try:
        executor.spin()

    except KeyboardInterrupt:
        node.get_logger().info('Traffic Controller shutting down...')

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()