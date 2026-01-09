import omni.isaac.motion_generation as mg
from omni.isaac.core.articulations import Articulation


class RMPFlowController(mg.MotionPolicyController):
    def __init__(self, name: str, robot_articulation: Articulation, physics_dt: float = 1.0 / 60.0) -> None:
        # 获取当前脚本所在目录
        import os
        current_dir = os.path.dirname(os.path.abspath(__file__))
        
        # 使用相对路径
        robot_description_path = os.path.abspath(os.path.join(current_dir, "../rmpflow/robot_descriptor.yaml"))
        rmpflow_config_path = os.path.abspath(os.path.join(current_dir, "../rmpflow/skyentific_rmpflow_common.yaml"))
        # 注意：yellow_arm.urdf可能不存在于当前项目中，需要根据实际情况调整
        # 如果使用CR5机器人的URDF，可以使用以下路径
        urdf_path = os.path.abspath(os.path.join(current_dir, "../../isaac_sim_models/urdf/cr5_robot.urdf"))
        
        self.rmpflow = mg.lula.motion_policies.RmpFlow(robot_description_path=robot_description_path,
                                                        rmpflow_config_path=rmpflow_config_path,
                                                        urdf_path=urdf_path,
                                                        end_effector_frame_name="yellowarm_link7",  # 注意：这个可能需要根据实际机器人模型调整
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
