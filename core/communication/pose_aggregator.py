import threading

class PoseAggregator:
    def __init__(self):
        self.lock = threading.Lock()
        self.sum_x = 0.0
        self.sum_y = 0.0
        self.sum_z = 0.0
        self.count = 0

    def update_pose(self, pose, count):
        with self.lock:
            x, y, z = pose
            self.sum_x += x
            self.sum_y += y
            self.sum_z += z
            self.count += count

    def get_average_pose(self):
        with self.lock:
            if self.count == 0:
                return None
            avg_x = self.sum_x / self.count
            avg_y = self.sum_y / self.count
            avg_z = self.sum_z / self.count
            return avg_x, avg_y, avg_z