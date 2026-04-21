import os
import time
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import yaml

from ..core.types import RobotObservation
from ..modules.controllers import JointSmoother
from ..modules.kinematics import KinematicsEngine
from ..r3.gripper import GripperController
from ..r3.ik_policy import IKFailurePolicy

from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.dynamic_control import _dynamic_control


class ArkIsaacBridge:
    def __init__(self, config_path: str, world: Any, runtime_cfg: Optional[Dict[str, Any]] = None):
        with open(config_path, "r", encoding="utf-8-sig") as f:
            self.cfg = yaml.safe_load(f)
        self.world = world
        self.runtime_cfg = runtime_cfg or {}
        root = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))

        self._dc = _dynamic_control.acquire_dynamic_control_interface()
        self.smoother = JointSmoother(factor=0.1)

        init_cfg = self.cfg["robot"].get("initial_state", {})
        self.init_pos = np.array(init_cfg.get("position", [0.0, 0.0, 0.4]), dtype=np.float64)
        self.init_ori = np.array(init_cfg.get("orientation", [1.0, 0.0, 0.0, 0.0]), dtype=np.float64)

        usd_path = os.path.join(root, self.cfg["robot"]["usd_path"])
        prim_path = self.cfg["robot"]["prim_path"]
        if not self.world.stage.GetPrimAtPath(prim_path).IsValid():
            add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)

        self.robot = self.world.scene.add(Articulation(prim_path=prim_path, name=self.cfg["robot"]["name"]))
        self.arm_joints = list(self.cfg["action_space"]["arm"]["joints"])
        self.arm_limits: List[Tuple[float, float]] = []
        for j in self.arm_joints:
            lo, hi = self.cfg["action_space"]["arm"]["joint_limits"][j]
            self.arm_limits.append((float(lo), float(hi)))

        self.gripper_joint = self.cfg["action_space"]["gripper"]["joints"][0]
        g_lo, g_hi = self.cfg["action_space"]["gripper"]["joint_limits"][self.gripper_joint]

        k_cfg_path = os.path.join(root, "configs/kinematics_config.yaml")
        with open(k_cfg_path, "r", encoding="utf-8") as f:
            k_data = yaml.safe_load(f)
        self.ee_link_name = str(k_data.get("ee_link", "Link6"))

        urdf_p = os.path.join(root, self.cfg["robot"]["urdf_path"])
        self.kinematics = KinematicsEngine(k_cfg_path, urdf_p)

        ik_retries = int(self.runtime_cfg.get("ik", {}).get("max_retries", 3))
        self.ik_policy = IKFailurePolicy(max_retries=ik_retries)

        g_cfg = self.runtime_cfg.get("gripper", {})
        self.gripper_controller = GripperController(
            lower_rad=float(g_lo),
            upper_rad=float(g_hi),
            stall_error_threshold=float(g_cfg.get("stall_error_threshold", 0.03)),
            movement_threshold=float(g_cfg.get("movement_threshold", 0.0001)),
            stall_steps=int(g_cfg.get("stall_steps", 45)),
        )

        self.arm_indices: List[int] = []
        self.gripper_index: Optional[int] = None
        self.art_controller = None

    def initialize_physics(self) -> None:
        self.robot.initialize()
        self.robot.set_world_pose(position=self.init_pos, orientation=self.init_ori)

        self.arm_indices = [int(self.robot.get_dof_index(j)) for j in self.arm_joints]

        g_idx = self.robot.get_dof_index(self.gripper_joint)
        self.gripper_index = int(g_idx) if g_idx is not None and int(g_idx) >= 0 else None

        total_dofs = int(self.robot.num_dof)
        full_kps = np.zeros(total_dofs)
        full_kds = np.zeros(total_dofs)

        arm_kps = [1661.40, 1530.29, 1733.09, 397.69, 30.09, 30.09]
        arm_kds = [0.664, 0.612, 0.693, 0.159, 0.012, 0.012]
        for i, idx in enumerate(self.arm_indices):
            full_kps[idx] = arm_kps[i] if i < len(arm_kps) else 100.0
            full_kds[idx] = arm_kds[i] if i < len(arm_kds) else 1.0

        if self.gripper_index is not None:
            full_kps[self.gripper_index] = 250.0
            full_kds[self.gripper_index] = 4.0

        self.art_controller = self.robot.get_articulation_controller()
        self.art_controller.set_gains(kps=full_kps, kds=full_kds)

    def get_runtime_context(self) -> Dict[str, Any]:
        return {
            "arm_joint_count": len(self.arm_joints),
            "arm_joint_names": list(self.arm_joints),
            "arm_joint_limits_rad": list(self.arm_limits),
            "gripper_joint_name": self.gripper_joint,
            "gripper_joint_index": self.gripper_index,
            "gripper_limits_rad": self.cfg["action_space"]["gripper"]["joint_limits"][self.gripper_joint],
            "ee_link": self.ee_link_name,
        }

    def _get_ee_pose(self) -> np.ndarray:
        art_handle = self._dc.get_articulation(self.robot.prim_path)
        body_handle = self._dc.find_articulation_body(art_handle, self.ee_link_name)
        if body_handle == 0:
            return np.zeros(7, dtype=np.float64)
        pose = self._dc.get_rigid_body_pose(body_handle)
        return np.array([pose.p.x, pose.p.y, pose.p.z, pose.r.w, pose.r.x, pose.r.y, pose.r.z], dtype=np.float64)

    def _get_gripper_position(self) -> float:
        if self.gripper_index is None:
            return 0.0
        value = self.robot.get_joint_positions(joint_indices=[self.gripper_index])
        if value is None or len(value) == 0:
            return 0.0
        return float(value[0])

    def get_observation(self) -> RobotObservation:
        joint_pos = self.robot.get_joint_positions()
        ee_pose = self._get_ee_pose()
        return RobotObservation(
            joint_positions=np.array(joint_pos, dtype=np.float64),
            ee_pose=ee_pose,
            odom=np.zeros(3, dtype=np.float64),
            gripper_position=self._get_gripper_position(),
            timestamp_ms=int(time.time() * 1000),
        )

    def _resolve_arm_target(self, command: Any, current_q: np.ndarray) -> Tuple[np.ndarray, Dict[str, Any]]:
        info: Dict[str, Any] = {
            "mode": command.command_type,
            "success": True,
            "reason_code": "",
            "policy_state": "NORMAL",
            "retry_count": 0,
        }

        if command.command_type == "pose" and command.pose_target_wxyz is not None:
            result = self.kinematics.get_joints(np.array(command.pose_target_wxyz), current_q)
            if result is not None:
                decision = self.ik_policy.on_success([float(v) for v in result])
                info.update(
                    {
                        "success": True,
                        "policy_state": decision.state,
                        "retry_count": decision.retry_count,
                    }
                )
                return np.array(decision.target_joint_rad, dtype=np.float64), info

            decision = self.ik_policy.on_failure([float(v) for v in current_q])
            info.update(
                {
                    "success": False,
                    "reason_code": decision.reason_code,
                    "policy_state": decision.state,
                    "retry_count": decision.retry_count,
                }
            )
            return np.array(decision.target_joint_rad, dtype=np.float64), info

        if command.command_type == "arm" and command.arm_target_rad is not None:
            target = np.array(command.arm_target_rad, dtype=np.float64)
            decision = self.ik_policy.on_success([float(v) for v in target])
            info.update(
                {
                    "success": True,
                    "policy_state": decision.state,
                    "retry_count": decision.retry_count,
                }
            )
            return target, info

        # For gripper/system commands hold arm position.
        return current_q, info

    def _resolve_gripper_target(self, command: Any, current_gripper: float) -> Dict[str, Any]:
        if command.command_type == "gripper":
            self.gripper_controller.set_command(
                mode=str(command.gripper_mode),
                position_rad=command.gripper_position_rad,
                feedback_rad=current_gripper,
            )
        elif command.command_type == "system" and command.system_operation == "reset":
            self.gripper_controller.set_command("open", None, current_gripper)

        status = self.gripper_controller.step(current_gripper)
        return {
            "mode": status.mode,
            "target_rad": status.target_rad,
            "feedback_rad": status.feedback_rad,
            "stalled": status.stalled,
            "reason_code": status.reason_code,
        }

    def step(self, command: Any) -> Tuple[RobotObservation, Dict[str, Any]]:
        current_q = self.robot.get_joint_positions(joint_indices=self.arm_indices)
        current_q = np.array(current_q, dtype=np.float64)

        target_q, ik_info = self._resolve_arm_target(command, current_q)
        safe_q = np.array(self.smoother.step(target_q), dtype=np.float64)

        current_gripper = self._get_gripper_position()
        gripper_info = self._resolve_gripper_target(command, current_gripper)

        joint_indices = list(self.arm_indices)
        joint_positions = safe_q.tolist()
        if self.gripper_index is not None:
            joint_indices.append(self.gripper_index)
            joint_positions.append(float(gripper_info["target_rad"]))

        self.art_controller.apply_action(
            ArticulationAction(
                joint_positions=np.array(joint_positions, dtype=np.float64),
                joint_indices=joint_indices,
            )
        )

        obs = self.get_observation()
        execution_info = {
            "ik": ik_info,
            "gripper": gripper_info,
            "target_arm_rad": target_q.tolist(),
            "applied_arm_rad": safe_q.tolist(),
        }
        return obs, execution_info
