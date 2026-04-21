import numpy as np
import yaml

from omni.isaac.motion_generation import LulaKinematicsSolver


class KinematicsEngine:
    def __init__(self, config_path: str, urdf_path: str):
        with open(config_path, "r", encoding="utf-8") as f:
            k_cfg = yaml.safe_load(f)

        self.ee_frame = str(k_cfg.get("ee_link", "link_6"))
        self.solver = LulaKinematicsSolver(robot_description_path=config_path, urdf_path=urdf_path)

        valid_frames = list(self.solver.get_all_frame_names())
        print("\n" + "=" * 50)
        print("[Kinematics] Lula available frames:")
        for frame in valid_frames:
            print(f"  - {frame}")

        if self.ee_frame not in valid_frames and valid_frames:
            print(f"[Kinematics][Warning] Configured ee_link '{self.ee_frame}' not found.")
            self.ee_frame = valid_frames[-1]
            print(f"[Kinematics][Info] Fallback ee_link set to '{self.ee_frame}'.")
        print("=" * 50 + "\n")

    def get_joints(self, target_pose: np.ndarray, current_joints: np.ndarray):
        target_pos = np.asarray(target_pose[:3], dtype=np.float64)
        target_quat = np.asarray(target_pose[3:], dtype=np.float64)

        joint_targets, success = self.solver.compute_inverse_kinematics(
            frame_name=self.ee_frame,
            target_position=target_pos,
            target_orientation=target_quat,
            warm_start=np.asarray(current_joints, dtype=np.float64),
        )

        if not success or joint_targets is None:
            return None

        target = np.asarray(joint_targets, dtype=np.float64)
        if target.shape[0] > np.asarray(current_joints).shape[0]:
            target = target[: np.asarray(current_joints).shape[0]]
        return target
