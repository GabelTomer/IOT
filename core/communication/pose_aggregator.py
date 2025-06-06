import threading

class PoseAggregator:
    def __init__(self):
        self.lock = threading.Lock()
        self.avg_x = None
        self.avg_y = None
        self.avg_z = None
        self.alpha = 0.05  # Smoothing factor

    def update_pose(self, pose):
        with self.lock:
            x, y, z = pose
            if self.avg_x is None:
                self.avg_x = x
                self.avg_y = y
                self.avg_z = z
            else:
                self.avg_x = (1 - self.alpha) * self.avg_x + self.alpha * x
                self.avg_y = (1 - self.alpha) * self.avg_y + self.alpha * y
                self.avg_z = (1 - self.alpha) * self.avg_z + self.alpha * z

    def get_average_pose(self):
        with self.lock:
            if self.avg_x is None:
                return None
            return self.avg_x, self.avg_y, self.avg_z