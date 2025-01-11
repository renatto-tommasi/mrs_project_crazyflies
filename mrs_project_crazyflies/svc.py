from nav_msgs.msg import OccupancyGrid
import numpy as np

class StateValidityChecker:

    def __init__(self, node, robot_dim=(1, 1)): # Pass the node instance
        self.node = node # Store the node instance
        self.map = None
        self.map_dim = None
        self.robot_dim = robot_dim
        self.map_resolution = None
        self.origin = None

        # Create a subscription using the provided node
        self.map_subscription = self.node.create_subscription(
            OccupancyGrid,
            '/map',
            self.save_map,
            10)  # QoS profile

    def save_map(self, msg: OccupancyGrid):
        self.map = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        self.map_dim = (msg.info.height, msg.info.width)
        self.map_resolution = msg.info.resolution
        self.origin = (msg.info.origin.position.x, msg.info.origin.position.y)

    def is_state_valid(self, state):
        if self.map is None:
            return False

        x, y = state
        row, col = self.map_to_grid(x, y)

        if self.get_cell_value(row, col) == 0:
            return self.check_robot_footprint(row, col)
        return False

    def map_to_grid(self, x, y):
        col = int((x - self.origin[0]) / self.map_resolution)
        row = int((y - self.origin[1]) / self.map_resolution)
        return (row, col)

    def get_cell_value(self, row, col):
        if self.map is None:  # Add check if map is None
            return 100 # or handle appropriately
        if 0 <= row < self.map_dim[0] and 0 <= col < self.map_dim[1]:
            return self.map[row * self.map_dim[1] + col]
        else:
            return 100

    def check_robot_footprint(self, row, col):
        if self.map is None: # Add check if map is None
            return False # or handle appropriately
        robot_rows = int(np.ceil(self.robot_dim[0] / self.map_resolution))
        robot_cols = int(np.ceil(self.robot_dim[1] / self.map_resolution))

        for i in range(max(0, row - robot_rows // 2), min(self.map_dim[0], row + robot_rows // 2 + 1)): #Added boundaries checks
            for j in range(max(0, col - robot_cols // 2), min(self.map_dim[1], col + robot_cols // 2 + 1)): #Added boundaries checks
                if self.get_cell_value(i, j) != 0:
                    return False
        return True