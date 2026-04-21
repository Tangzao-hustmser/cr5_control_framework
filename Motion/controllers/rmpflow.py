import isaacsim.robot_motion.motion_generation as mg
from isaacsim.core.prims import SingleArticulation
import os

# 尝试导入RmpFlow类
import logging

# 尝试构建一个稳健的导入链
try:
    # 优先级 1: 标准 RmpFlow 路径
    from isaacsim.robot_motion.motion_generation.lula import RmpFlow
    print("Using RmpFlow as the primary motion policy.")

except ImportError:
    try:
        # 优先级 2: 直接通过 lula 库导入
        import lula
        RmpFlow = lula.motion_policies.RmpFlow
        print("Using RmpFlow from direct lula bindings.")
        
    except (ImportError, AttributeError):
        # 优先级 3: 备选方案 - Lula 逆运动学策略 (ArticulatedLulaPosePolicy)
        # 这是 Isaac Sim 中除了 RMPFlow 外最常用的运动生成器
        try:
            from isaacsim.robot_motion.motion_generation.lula import ArticulatedLulaPosePolicy as RmpFlow
            print("RmpFlow not found. Falling back to ArticulatedLulaPosePolicy.")
        except ImportError:
            # 优先级 4: 最后的保底 - 提示用户或使用简单的纯 IK
            logging.error("No valid motion generation library found. Please check Isaac Sim installation.")
            # 如果这里必须赋值，则赋予一个能运行的最简接口类
            from isaacsim.robot_motion.motion_generation import MotionPolicyInterface
            RmpFlow = MotionPolicyInterface

class RMPFlowController(mg.MotionPolicyController):
    def __init__(self, name: str, robot_articulation: SingleArticulation, physics_dt: float = 1.0 / 60.0) -> None:
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
