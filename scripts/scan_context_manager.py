import numpy as np
import os
from utils import kdtree

np.set_printoptions(precision=4)


class RingKeyWithIndex(object):
    def __init__(self, ring_key, index):
        self.ring_key = ring_key
        self.index = index

    def __len__(self):
        return len(self.ring_key)

    def __getitem__(self, item):
        return self.ring_key[item]

    def __repr__(self):
        return "RingKey{} {}".format(self.ring_key, self.index)


class ScanContextManager:
    def __init__(self, shape=None, num_candidates=10, threshold=0.6, file_path=None):
        if shape is None:
            shape = [20, 60]
        self.shape = shape

        if file_path is None:
            file_path = "/data/slam_data/"
        self.file_path = file_path

        self.num_candidates = num_candidates
        self.threshold = threshold

        self.max_length = 80
        self.ENOUGH_LARGE = 15000  # capable of up to ENOUGH_LARGE number of nodes
        self.pt_clouds = [None] * self.ENOUGH_LARGE
        self.scan_contexts = [None] * self.ENOUGH_LARGE
        self.ring_keys = [None] * self.ENOUGH_LARGE
        self.curr_node_idx = 0
        self.poses = [None] * self.ENOUGH_LARGE

    def save(self):
        self.save_sc()
        self.save_ring_key()
        self.save_pose()

    def add_node(self, node_idx, pt_cloud):
        sc = ScanContextManager.pt_cloud_to_sc(
            pt_cloud, self.shape, self.max_length)
        ring_key = ScanContextManager.scan_context_to_ring_key(sc)

        self.curr_node_idx = node_idx
        self.pt_clouds[node_idx] = pt_cloud
        self.scan_contexts[node_idx] = sc
        self.ring_keys[node_idx] = RingKeyWithIndex(ring_key, node_idx)

    def get_pt_cloud(self, node_idx):
        return self.pt_clouds[node_idx]

    def initialization(self, pt_cloud):
        from utils.point_cloud_utils import random_sampling
        pt_cloud = random_sampling(pt_cloud, num_points=15000)
        sc = ScanContextManager.pt_cloud_to_sc(pt_cloud
                                               , self.shape, self.max_length)
        ring_key = ScanContextManager.scan_context_to_ring_key(sc)

        idx, dist, yaw_diff_deg = self.__query_loop_closure(
            sc, ring_key, self.curr_node_idx)

        if idx is None:
            print("No matched index found!")
            return None
        else:
            return self.livox_init_pose(idx, pt_cloud[:, :3], yaw_diff_deg)

    def livox_init_pose(self, matched_idx, curr_scan, yaw_diff_angle):
        from utils.point_cloud_utils import random_sampling
        from utils import icp_utlis as ICP
        pt_cloud_file = os.path.join(
            self.file_path, "pcd_npy") + "/pc_" + str(matched_idx) + ".npy"

        pt_cloud = np.load(pt_cloud_file)[:, :3]

        num_points = min(len(pt_cloud), len(curr_scan))
        pt_cloud_downsampled = random_sampling(pt_cloud, num_points=num_points)
        curr_scan_downsampled = random_sampling(curr_scan, num_points=num_points)

        loop_trans, distance, _ = ICP.icp(curr_scan_downsampled, pt_cloud_downsampled,
                                          init_pose=self.yawdeg2se3(yaw_diff_angle),
                                          max_iterations=20)

        # Load LiDAR Mapping related pose
        pose_file = os.path.join(self.file_path, "pose.npy")
        print("Load Pose history from: ", pose_file)
        pose_list = ScanContextManager.livox_load_pose(pose_file)

        sc_pose = pose_list[matched_idx]
        return np.matmul(sc_pose, loop_trans)

    @staticmethod
    def livox_load_pose(pose_file):
        return np.load(pose_file)

    @staticmethod
    def livox_load_odom_pose(pose_file):
        se3 = np.eye(4)
        with open(pose_file, "r") as f:
            for i, line in enumerate(f):
                result = [ele for ele in line.split(" ") if ele.strip()]
                se3[i, 0] = float(result[0])
                se3[i, 1] = float(result[1])
                se3[i, 2] = float(result[2])
                se3[i, 3] = float(result[3])
        return se3

    def livox_load_pc_make_sc(self, pc_path):
        import open3d as o3d

        pc_files = [f for f in os.listdir(pc_path) if ".pcd" in f]
        pc_files = sorted(pc_files, key=lambda pc_file: int(
            pc_file[:pc_file.find("_")]))

        odom_files = [f for f in os.listdir(pc_path) if ".odom" in f]
        odom_files = sorted(odom_files, key=lambda odom_file: int(
            odom_file[:odom_file.find("_")]))

        pcd_npy_folder = os.path.join(self.file_path, "pcd_npy")
        if not os.path.exists(pcd_npy_folder):
            os.makedirs(pcd_npy_folder)

        node_idx = 0
        for pc, pose_name in zip(pc_files, odom_files):
            full_path = os.path.join(pc_path, pc)
            pc_numpy = np.asarray(o3d.io.read_point_cloud(filename=full_path).points)
            self.add_node(node_idx, pc_numpy)

            npy_file_name = os.path.join(pcd_npy_folder, "pc_" + str(node_idx) + ".npy")
            np.save(npy_file_name, pc_numpy)

            pose_file = os.path.join(pc_path, pose_name)
            pose_se3 = ScanContextManager.livox_load_odom_pose(pose_file)
            self.poses[node_idx] = pose_se3
            node_idx += 1

        self.save()

    def livox_load_sc_rk(self, sc_path=None, rk_path=None):
        if sc_path is None:
            sc_path = os.path.join(self.file_path, "scancontext")
        if rk_path is None:
            rk_path = os.path.join(self.file_path, "ringkey")

        self.load_sc(sc_path)
        self.load_ring_key(rk_path)
        print("Load SC and RK successfully!")

    def __query_loop_closure(self, query_scan_context, query_ring_key, curr_idx):

        ring_key_history = self.ring_keys[: curr_idx]
        ring_key_tree = kdtree.create(ring_key_history)

        nn_candidates_idx = ring_key_tree.search_knn(
            query_ring_key.tolist(), k=self.num_candidates)

        idx_list = []
        for nn_candidates in nn_candidates_idx:
            if nn_candidates[1] < self.threshold:
                idx_list.append(nn_candidates[0].data.index)

        nn_dist = 1.0
        nn_idx = None
        nn_yaw_diff = None
        for i in range(len(idx_list)):
            candidate_idx = idx_list[i]
            candidate_sc = self.scan_contexts[candidate_idx]

            dist, yaw_diff = ScanContextManager.sc_distance(
                candidate_sc, query_scan_context)

            if dist < nn_dist:
                nn_dist = dist
                nn_yaw_diff = yaw_diff
                nn_idx = candidate_idx

        if nn_dist < self.threshold:
            nn_yaw_diff_deg = nn_yaw_diff * (360 / self.shape[1])
            return nn_idx, nn_dist, nn_yaw_diff_deg
        else:
            return None, None, None

    def detect_loop_closure(self):
        exclude_recent_nodes = 30

        valid_recent_node_idx = self.curr_node_idx - exclude_recent_nodes

        # avoid query SC too early
        if valid_recent_node_idx < 1:
            return None, None, None
        else:
            query_sc = self.scan_contexts[self.curr_node_idx]
            ring_key_query = self.ring_keys[self.curr_node_idx].ring_key

            return self.__query_loop_closure(query_sc, ring_key_query, valid_recent_node_idx)

    def save_pose(self):
        pose_file_path = os.path.join(self.file_path, "pose.npy")
        print("Save Pose information to: ", pose_file_path)
        np.save(pose_file_path, np.asarray(self.poses[: self.curr_node_idx]))

    def save_sc(self):
        sc_file_path = os.path.join(self.file_path, "scancontext")
        print("Save ScanContext to: ", sc_file_path)

        if not os.path.exists(sc_file_path):
            os.makedirs(sc_file_path)
        for i in range(self.curr_node_idx + 1):
            sc_file_name = "sc_" + str(i) + ".npy"
            np.save(os.path.join(sc_file_path, sc_file_name),
                    self.scan_contexts[i])

    def load_sc(self, sc_file_path=None):
        if sc_file_path is None:
            raise RuntimeError("SC File Path Not Exist")

        sc_files = os.listdir(sc_file_path)
        sc_files = sorted(sc_files, key=lambda file: int(
            file[file.find("_") + 1: file.find(".")]))
        self.curr_node_idx = len(sc_files)

        idx = 0
        for sc_file in sc_files:
            sc = np.load(os.path.join(sc_file_path, sc_file))
            self.scan_contexts[idx] = sc
            idx += 1

    def save_ring_key(self):
        ring_key_path = os.path.join(self.file_path, "ringkey")

        if not os.path.exists(ring_key_path):
            os.makedirs(ring_key_path)

        for i in range(self.curr_node_idx + 1):
            rk_file_name = "rk_" + str(i) + ".npy"
            np.save(os.path.join(ring_key_path, rk_file_name),
                    self.ring_keys[i].ring_key)

    def load_ring_key(self, ring_key_path=None):
        if ring_key_path is None:
            raise RuntimeError("RK File Path Not Exist")

        rk_files = os.listdir(ring_key_path)
        rk_files = sorted(rk_files, key=lambda file: int(
            file[file.find("_") + 1: file.find(".")]))

        idx = 0
        for rk_file in rk_files:
            rk_file_name = os.path.join(ring_key_path, rk_file)
            ring_key = np.load(os.path.join(ring_key_path, rk_file_name))
            self.ring_keys[idx] = RingKeyWithIndex(ring_key, idx)
            idx += 1

    @staticmethod
    def xyzrpy2se3(pose):
        se3 = np.eye(4)
        theta = [float(pose[3]), float(pose[4]), float(pose[5])]
        se3[:3, :3] = ScanContextManager.eulerAnglesToRotationMatrix(theta)
        se3[0, 3] = pose[0]
        se3[1, 3] = pose[1]
        se3[2, 3] = pose[2]
        return se3

    @staticmethod
    def yawdeg2se3(yaw_deg):
        se3 = np.eye(4)
        se3[:3, :3] = ScanContextManager.yawdeg2so3(yaw_deg)
        return se3

    @staticmethod
    def yawdeg2so3(yaw_deg):
        yaw_rad = np.deg2rad(yaw_deg)
        return ScanContextManager.eulerAnglesToRotationMatrix([0, 0, yaw_rad])

    @staticmethod
    def eulerAnglesToRotationMatrix(theta):

        R_x = np.array([[1, 0, 0],
                        [0, np.cos(theta[0]), -np.sin(theta[0])],
                        [0, np.sin(theta[0]), np.cos(theta[0])]
                        ])

        R_y = np.array([[np.cos(theta[1]), 0, np.sin(theta[1])],
                        [0, 1, 0],
                        [-np.sin(theta[1]), 0, np.cos(theta[1])]
                        ])

        R_z = np.array([[np.cos(theta[2]), -np.sin(theta[2]), 0],
                        [np.sin(theta[2]), np.cos(theta[2]), 0],
                        [0, 0, 1]
                        ])

        R = np.dot(R_z, np.dot(R_y, R_x))

        return R

    @staticmethod
    def sc_distance(sc1, sc2):
        num_sectors = sc1.shape[1]

        # repeat to move 1 column
        _one_step = 1

        sim_for_each_col = np.zeros(num_sectors)
        for i in range(num_sectors):
            # shift
            sc1 = np.roll(sc1, _one_step, axis=1)

            # compare
            sum_of_cossim = 0
            num_col_engaged = 0

            for j in range(num_sectors):
                col_j_1 = sc1[:, j]
                col_j_2 = sc2[:, j]

                norm_col_j_1 = np.linalg.norm(col_j_1)
                norm_col_j_2 = np.linalg.norm(col_j_2)

                if not norm_col_j_1 or not norm_col_j_2:
                    continue

                cos_sim = np.dot(col_j_1, col_j_2) / \
                          (norm_col_j_1 * norm_col_j_2)
                sum_of_cossim += cos_sim
                num_col_engaged += 1

            if num_col_engaged == 0:
                sim_for_each_col[i] = 0
                continue

            sim_for_each_col[i] = sum_of_cossim / num_col_engaged

        yaw_diff = np.argmax(sim_for_each_col) + 1  # NOTE Python starts from 0
        sim = np.max(sim_for_each_col)
        dist = 1 - sim

        return dist, yaw_diff

    @staticmethod
    def pt_cloud_to_sc(pt_cloud, sc_shape, max_length):
        num_ring = sc_shape[0]
        num_sector = sc_shape[1]

        gap_ring = int(max_length) / int(num_ring)
        gap_sector = 360 / num_sector  # TODO to update the FOV for LIVOX

        ENOUGH_LARGE = 500
        GROUND_HEIGHT = 2.0

        sc_storge = np.zeros([ENOUGH_LARGE, num_ring, num_sector])
        sc_counter = np.zeros([num_ring, num_sector])

        num_pts = pt_cloud.shape[0]
        for pt_idx in range(num_pts):
            point = pt_cloud[pt_idx, :]

            point_height = point[2] + GROUND_HEIGHT
            idx_ring, idx_sector = ScanContextManager.pt_2_ring_sector(point, gap_ring, gap_sector, num_ring,
                                                                       num_sector)
            if sc_counter[idx_ring, idx_sector] >= ENOUGH_LARGE:
                continue

            sc_storge[int(sc_counter[idx_ring, idx_sector]),
                      idx_ring, idx_sector] = point_height
            sc_counter[idx_ring,
                       idx_sector] = sc_counter[idx_ring, idx_sector] + 1

        sc = np.amax(sc_storge, axis=0)

        return sc

    @staticmethod
    def scan_context_to_ring_key(scan_context):
        return np.mean(scan_context, axis=1)

    @staticmethod
    def xy2theta(x, y):
        theta = 0
        if x >= 0 and y >= 0:
            theta = 180 / np.pi * np.arctan(y / x)
        if x < 0 <= y:
            theta = 180 - ((180 / np.pi) * np.arctan(y / (-x)))
        if x < 0 and y < 0:
            theta = 180 + ((180 / np.pi) * np.arctan(y / x))
        if x >= 0 > y:
            theta = 360 - ((180 / np.pi) * np.arctan((-y) / x))

        return theta

    @staticmethod
    def pt_2_ring_sector(point, gap_ring, gap_sector, num_ring, num_sector):
        x = point[0]
        y = point[1]

        if x == 0.0:
            x = 0.001
        if y == 0.0:
            y = 0.001

        theta = ScanContextManager.xy2theta(x, y)
        faraway = np.sqrt(x * x + y * y)

        idx_ring = int(faraway // gap_ring)
        idx_sector = int(theta // gap_sector)

        if idx_ring >= num_ring:
            idx_ring = num_ring - 1  # python starts with 0 and ends with N-1

        return int(idx_ring), int(idx_sector)
