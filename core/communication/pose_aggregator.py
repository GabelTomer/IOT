import threading

class PoseAggregator:
    def __init__(self):
        self.lock = threading.Lock()
        self.poses = {}  # key: client_id, value: (x, y, z)
        self.main_pose = None  # the main Pi's pose

    def update_remote_pose(self, client_id, pose):
        with self.lock:
            self.poses[client_id] = pose

    def update_main_pose(self, pose):
        with self.lock:
            self.main_pose = pose

    def get_average_pose(self):
        with self.lock:
            all_poses = list(self.poses.values())
            if self.main_pose is not None:
                all_poses.append(self.main_pose)
            if not all_poses:
                return None
            x, y, z = zip(*all_poses)
            return (
                sum(x) / len(x),
                sum(y) / len(y),
                sum(z) / len(z)
            )
    def remove_remote_pose(self, client_id):
        with self.lock:
            self.poses.pop(client_id, None)  # remove if exists, do nothing otherwise