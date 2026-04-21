from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})
import os
import sys

# 获取脚本所在目录并添加到Python路径
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.append(script_dir)

<<<<<<< HEAD
# 添加mytasks和controllers目录到Python路径
tasks_dir = os.path.join(script_dir, "mytasks")
controllers_dir = os.path.join(script_dir, "controllers")
if tasks_dir not in sys.path:
    sys.path.append(tasks_dir)
if controllers_dir not in sys.path:
    sys.path.append(controllers_dir)

# 确保工作目录是脚本所在目录
os.chdir(script_dir)

from omni.isaac.core import World
# 直接导入模块
from mytasks.follow_target import FollowTarget
=======
# 确保工作目录是脚本所在目录
os.chdir(script_dir)

from isaacsim.core.api import World
from tasks.follow_target import FollowTarget
>>>>>>> 12d2d10 (2026.1.25)
import numpy as np
from controllers.rmpflow import RMPFlowController
import omni.graph.core as og


# 创建世界
my_world = World(stage_units_in_meters=1.0)
<<<<<<< HEAD

# 初始化Follow Target任务
=======
#Initialize the Follow Target task with a target location for the cube to be followed by the end effector
# 可以通过robot_position和robot_orientation参数设置机器人的初始位置偏移
# 例如，将机器人在x轴方向偏移0.5米
>>>>>>> 12d2d10 (2026.1.25)
my_task = FollowTarget(
    name="skyentific_follow_target", 
    target_position=np.array([0.5, 0, 0.6]), 
    robot_position=np.array([0, 0, 0.3])
)
my_world.add_task(my_task)
my_world.reset()

# 获取任务参数
task_params = my_world.get_task("skyentific_follow_target").get_params()
target_name = task_params["target_name"]["value"]
skyentific_name = task_params["robot_name"]["value"]

# 获取机器人对象
my_skyentific = my_world.scene.get_object(skyentific_name)

# 初始化控制器
my_controller = RMPFlowController(
    name="target_follower_controller", 
    robot_articulation=my_skyentific,
    physics_dt=1.0 / 60.0  # 提高控制频率
)
my_controller.reset()

# 获取关节控制器
articulation_controller = my_skyentific.get_articulation_controller()

while simulation_app.is_running():
    my_world.step(render=True)
    
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()
            my_controller.reset()
        
        # 获取观察数据
        observations = my_world.get_observations()
        
        # 计算并应用机器人手臂控制动作
        actions = my_controller.forward(
            target_end_effector_position=observations[target_name]["position"],
            target_end_effector_orientation=observations[target_name]["orientation"],
        )
        articulation_controller.apply_action(actions)
<<<<<<< HEAD
        
        # 打印CR5机器人的关节位置（只有6个关节，索引0-5）
        print(observations["cr5_robot"]["joint_positions"][:6])
        current_position = my_skyentific.end_effector.get_world_pose()[0]
        target_position = observations[target_name]["position"]
        error = np.linalg.norm(current_position - target_position)
        print(f"跟随误差: {error:.3f} meters")

# 关闭应用程序
=======
        # 打印CR5机器人的关节位置（只有6个关节，索引0-5）
        print(observations["cr5_robot"]["joint_positions"]) # 打印所有关节位置
>>>>>>> 12d2d10 (2026.1.25)
simulation_app.close()
