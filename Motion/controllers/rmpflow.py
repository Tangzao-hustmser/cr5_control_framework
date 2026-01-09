import omni.isaac.motion_generation as mg
from omni.isaac.core.articulations import Articulation
import os

# 尝试导入RmpFlow类
try:
    from omni.isaac.motion_generation.lula import RmpFlow
except ImportError:
    try:
        import lula
        RmpFlow = lula.motion_policies.RmpFlow
    except AttributeError:
        # 如果以上都失败，尝试使用MotionPolicyInterface作为替代
        from omni.isaac.motion_generation import MotionPolicyInterface
        RmpFlow = MotionPolicyInterface


class RMPFlowController(mg.MotionPolicyController):
    def __init__(self, name: str, robot_articulation: Articulation, physics_dt: float = 1.0 / 60.0) -> None:
        # 获取脚本所在目录
        script_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        
        # 使用正确的路径
        robot_description_path = os.path.join(script_dir, "rmpflow", "robot_descriptor.yaml")
        rmpflow_config_path = os.path.join(script_dir, "rmpflow", "skyentific_rmpflow_common.yaml")
        urdf_path = os.path.join(script_dir, "..", "isaac_sim_models", "urdf", "cr5_robot.urdf")
        
        # 初始化RMPFlow
        self.rmpflow = RmpFlow(robot_description_path=robot_description_path,
                              rmpflow_config_path=rmpflow_config_path,
                              urdf_path=urdf_path,
                              end_effector_frame_name="Link6",
                              maximum_substep_size=0.00334)
        self.articulation_rmp = mg.ArticulationMotionPolicy(robot_articulation, self.rmpflow, physics_dt)

        mg.MotionPolicyController.__init__(self, name=name, articulation_motion_policy=self.articulation_rmp)
        self._default_position, self._default_orientation = (
            self._articulation_motion_policy._robot_articulation.get_world_pose()
        )
        self._motion_policy.set_robot_base_pose(
            robot_position=self._default_position, robot_orientation=self._default_orientation
        )
        return

    def reset(self):
        mg.MotionPolicyController.reset(self)
        self._motion_policy.set_robot_base_pose(
            robot_position=self._default_position, robot_orientation=self._default_orientation
        )
