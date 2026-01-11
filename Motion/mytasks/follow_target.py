from isaacsim.robot.manipulators import SingleManipulator
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.core.utils.stage import add_reference_to_stage
import isaacsim.core.api.tasks as tasks
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
            gripper=gripper,
            position=self._robot_position,
            orientation=self._robot_orientation
        )
        
        # 设置默认关节位置（22个自由度）
        joints_default_positions = np.zeros(22)
        manipulator.set_joints_default_state(positions=joints_default_positions)
        
        return manipulator
        
    def _move_task_objects_to_their_frame(self):
        """重写方法，确保只对机器人根关节应用变换，避免非根关节变换警告"""
        for object_name, task_object in self._task_objects.items():
            # 只对机器人根关节应用变换，跳过其他部件
            if hasattr(task_object, 'set_world_pose') and hasattr(task_object, 'get_world_pose'):
                # 检查是否是机器人根关节
                if hasattr(task_object, 'prim_path') and task_object.prim_path == "/World/cr5_robot":
                    current_position, current_orientation = task_object.get_world_pose()
                    task_object.set_world_pose(position=current_position + self._offset)
                    task_object.set_default_state(position=current_position + self._offset)
        return
