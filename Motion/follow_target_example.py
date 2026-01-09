from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})
import os
import sys

# 获取脚本所在目录并添加到Python路径
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.append(script_dir)

# 确保工作目录是脚本所在目录
os.chdir(script_dir)

from omni.isaac.core import World
from tasks.follow_target import FollowTarget
import numpy as np
from controllers.rmpflow import RMPFlowController

my_world = World(stage_units_in_meters=1.0)
#Initialize the Follow Target task with a target location for the cube to be followed by the end effector
# 可以通过robot_position和robot_orientation参数设置机器人的初始位置偏移
# 例如，将机器人在x轴方向偏移0.5米
my_task = FollowTarget(
    name="skyentific_follow_target", 
    target_position=np.array([0.5, 0, 1.0]),
    robot_position=np.array([0, 0, 0.5])  # 添加位置偏移
)
my_world.add_task(my_task)
my_world.reset()
task_params = my_world.get_task("skyentific_follow_target").get_params()
target_name = task_params["target_name"]["value"]
skyentific_name = task_params["robot_name"]["value"]
my_skyentific = my_world.scene.get_object(skyentific_name)
#initialize the controller
my_controller = RMPFlowController(name="target_follower_controller", robot_articulation=my_skyentific)
my_controller.reset()
articulation_controller = my_skyentific.get_articulation_controller()
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()
            my_controller.reset()
        observations = my_world.get_observations()
        actions = my_controller.forward(
            target_end_effector_position=observations[target_name]["position"],
            target_end_effector_orientation=observations[target_name]["orientation"],
            )
        articulation_controller.apply_action(actions)
        # 打印CR5机器人的关节位置（只有6个关节，索引0-5）
        print(observations["cr5_robot"]["joint_positions"]) # 打印所有关节位置
simulation_app.close()
