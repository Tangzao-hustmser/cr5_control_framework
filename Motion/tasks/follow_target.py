from omni.isaac.manipulators import SingleManipulator
from omni.isaac.manipulators.grippers import ParallelGripper
from omni.isaac.core.utils.stage import add_reference_to_stage
import omni.isaac.core.tasks as tasks
from typing import Optional
import numpy as np
import os


# Inheriting from the base class Follow Target
class FollowTarget(tasks.FollowTarget):
    def __init__(
        self,
        name: str = "skyentific_follow_target",
        target_prim_path: Optional[str] = None,
        target_name: Optional[str] = None,
        target_position: Optional[np.ndarray] = None,
        target_orientation: Optional[np.ndarray] = None,
        offset: Optional[np.ndarray] = None,
        robot_position: Optional[np.ndarray] = None,
        robot_orientation: Optional[np.ndarray] = None,
    ) -> None:
        tasks.FollowTarget.__init__(
            self,
            name=name,
            target_prim_path=target_prim_path,
            target_name=target_name,
            target_position=target_position,
            target_orientation=target_orientation,
            offset=offset,
        )
        self._robot_position = robot_position
        self._robot_orientation = robot_orientation
        return

    def set_robot(self) -> SingleManipulator:
        # 使用CR5机器人的USD文件（从urdf转换而来）
        # 获取当前脚本所在目录
        current_dir = os.path.dirname(os.path.abspath(__file__))
        # 构建相对于当前脚本的USD文件路径
        asset_path = os.path.abspath(os.path.join(current_dir, "../../isaac_sim_models/urdf/ranger_mini_cr5_with_gripper/ranger_mini_cr5_with_gripper.usd"))
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/cr5_robot")
        
        # 配置夹爪
        gripper = ParallelGripper(
            end_effector_prim_path="/World/cr5_robot/dh_robotics_ag95_base_link",
            joint_prim_names=["finger_joint", "right_crank_joint"],
            joint_opened_positions=np.array([0.0, 0.0]),
            joint_closed_positions=np.array([0.6524, 0.6524]),
            action_deltas=np.array([0.6524, 0.6524]),
        )
        
        # 定义机械臂
        manipulator = SingleManipulator(
            prim_path="/World/cr5_robot", 
            name="cr5_robot",
            end_effector_prim_path="/World/cr5_robot/dh_robotics_ag95_base_link",
            gripper=gripper
        )
        
        # 设置默认关节位置（22个自由度）
        joints_default_positions = np.zeros(22)
        manipulator.set_joints_default_state(positions=joints_default_positions)
        
        # 应用机器人位置偏移
        if self._robot_position is not None or self._robot_orientation is not None:
            manipulator.set_world_pose(position=self._robot_position, orientation=self._robot_orientation)
        
        return manipulator
