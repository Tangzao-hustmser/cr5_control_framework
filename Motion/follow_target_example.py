from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})
import os
import sys

# 获取脚本所在目录并添加到Python路径
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.append(script_dir)

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
import numpy as np
from controllers.rmpflow import RMPFlowController
import omni.graph.core as og


# 创建世界
my_world = World(stage_units_in_meters=1.0)

# 初始化Follow Target任务
my_task = FollowTarget(
    name="skyentific_follow_target", 
    target_position=np.array([0.5, 0, 1.0]),
    robot_position=np.array([0, 0, 0.5])  # 添加位置偏移
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
my_controller = RMPFlowController(name="target_follower_controller", robot_articulation=my_skyentific)
my_controller.reset()

# 获取关节控制器
articulation_controller = my_skyentific.get_articulation_controller()

# 编辑运动图

keys = og.Controller.Keys
(graph_handle, list_of_nodes, _, _) = og.Controller.edit(
    {"graph_path": "/action_graph", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("W", "omni.graph.nodes.ReadKeyboardState"),
            ("A", "omni.graph.nodes.ReadKeyboardState"),
            ("S", "omni.graph.nodes.ReadKeyboardState"),
            ("D", "omni.graph.nodes.ReadKeyboardState"),
            ("ToDoubleW","omni.graph.nodes.ToDouble"),
            ("ToDoubleA","omni.graph.nodes.ToDouble"),
            ("ToDoubleS","omni.graph.nodes.ToDouble"),
            ("ToDoubleD","omni.graph.nodes.ToDouble"),
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("DifferentialController", "isaacsim.robot.wheeled_robots.DifferentialController"),
            ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
            ("ArrayNames", "omni.graph.nodes.ConstructArray"),
            ("NegateLinear", "omni.graph.nodes.Multiply"),
            ("NegateAngular", "omni.graph.nodes.Multiply"),
            ("AddLinear", "omni.graph.nodes.Add"),
            ("AddAngular", "omni.graph.nodes.Add"),
            ("SpeedLinear", "omni.graph.nodes.ConstantDouble"),
            ("SpeedAngular", "omni.graph.nodes.ConstantDouble"),
            ("NegOne", "omni.graph.nodes.ConstantInt"),
        ],
        keys.SET_VALUES: [
              # setting the log level to warning so we can see the printout in terminal
        ],
        keys.CONNECT: [

        ],
    },
)


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
        
        # 打印CR5机器人的关节位置（只有6个关节，索引0-5）
        print(observations["cr5_robot"]["joint_positions"][:6])

# 关闭应用程序
simulation_app.close()
