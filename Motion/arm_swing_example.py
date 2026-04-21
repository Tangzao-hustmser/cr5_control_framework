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
from collections import deque

# 创建世界
my_world = World(stage_units_in_meters=1.0)

# 初始化任务
my_task = FollowTarget(
    name="skyentific_swing_arm", 
    target_position=np.array([0, 0, 0.5]), 
    robot_position=np.array([0, 0, 0.5])
)
my_world.add_task(my_task)
my_world.reset()

# 获取任务参数
task_params = my_world.get_task("skyentific_swing_arm").get_params()
target_name = task_params["target_name"]["value"]
skyentific_name = task_params["robot_name"]["value"]

# 获取机器人对象
my_skyentific = my_world.scene.get_object(skyentific_name)

# 初始化控制器
my_controller = RMPFlowController(
    name="arm_swing_controller", 
    robot_articulation=my_skyentific,
    physics_dt=1.0 / 60.0  # 提高控制频率
)
my_controller.reset()

# 获取关节控制器
articulation_controller = my_skyentific.get_articulation_controller()

# 调试信息
print(f"机器人名称: {skyentific_name}")
print(f"机器人类型: {type(my_skyentific)}")
print(f"末端执行器: {my_skyentific.end_effector}")
print(f"末端执行器prim路径: {my_skyentific.end_effector.prim_path if hasattr(my_skyentific.end_effector, 'prim_path') else 'N/A'}")

# 获取末端执行器初始位置
end_effector_pose = my_skyentific.end_effector.get_world_pose()
initial_position = end_effector_pose[0]
initial_orientation = end_effector_pose[1]
print(f"初始位置: {initial_position}")
print(f"初始姿态: {initial_orientation}")

# 检查控制器配置
print(f"控制器类型: {type(my_controller)}")
print(f"控制器配置完成")

# 摆动参数设置
# 根据机械臂初始位置调整目标位置
swing_amplitude = 0.1  # 摆动幅度（米）
swing_period = 2.0      # 摆动周期（秒）
swing_center = np.array([initial_position[0], initial_position[1], initial_position[2]])  # 以初始位置为摆动中心点
swing_axis = np.array([1, 0, 0])        # 摆动轴（X轴方向）

print(f"\n摆动参数设置:")
print(f"摆动中心点: {swing_center}")
print(f"摆动幅度: {swing_amplitude} 米")
print(f"摆动周期: {swing_period} 秒")
print(f"摆动轴: {swing_axis}")

# 时间跟踪变量
simulation_time = 0.0
time_step = 1.0 / 60.0  # 假设60Hz的仿真频率 

while simulation_app.is_running():
    my_world.step(render=True)
    
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()
            my_controller.reset()
            simulation_time = 0.0
            print("\n=== 仿真重置完成 ===")
            # 重置后的初始位置
            reset_pose = my_skyentific.end_effector.get_world_pose()
            print(f"重置后初始位置: {reset_pose[0]}")
        else:
            # 更新仿真时间
            simulation_time += time_step
        
        # 计算摆动目标位置
        # 使用7阶多项式轨迹规划，确保位置、速度、加速度连续
        def generate_smooth_trajectory(t, period, amplitude, center, axis):
    # 7阶多项式系数计算
            normalized_t = (t % period) / period
            if normalized_t < 0.5:
        # 上升阶段
                s = normalized_t * 2
        # 7阶多项式：s^3*(10s^4 - 15s^3 + 6s^2)
                swing_factor = s**3 * (10*s**4 - 15*s**3 + 6*s**2)
            else:
        # 下降阶段
                s = (1 - normalized_t) * 2
                swing_factor = s**3 * (10*s**4 - 15*s**3 + 6*s**2)
    
            swing_offset = amplitude * swing_factor
            return center + swing_offset * axis

        # 使用优化后的轨迹规划
        target_position = generate_smooth_trajectory(simulation_time, swing_period, swing_amplitude, swing_center, swing_axis)
        
        # 保持目标姿态不变
        # 获取当前末端执行器的姿态作为目标姿态
        current_pose = my_skyentific.end_effector.get_world_pose()
        current_position = current_pose[0]
        current_orientation = current_pose[1]
        
        # 计算并应用机器人手臂控制动作
        try:
            actions = my_controller.forward(
                target_end_effector_position=target_position,
                target_end_effector_orientation=current_orientation,
            )
            
            # 应用控制动作
            articulation_controller.apply_action(actions)
            
            # 获取应用动作后的实际位置
            new_position = my_skyentific.end_effector.get_world_pose()[0]
            error = np.linalg.norm(new_position - target_position)
            
            # 打印当前状态信息
            print(f"时间: {simulation_time:.2f}s, 目标位置: {target_position}, 当前位置: {new_position}, 误差: {error:.3f} meters")
            
            # 每5秒打印一次关节角度
            if int(simulation_time) % 5 == 0 and simulation_time - int(simulation_time) < time_step:
                observations = my_world.get_observations()
                if "cr5_robot" in observations:
                    joint_positions = observations["cr5_robot"]["joint_positions"][:6]
                    print(f"\n关节角度: {joint_positions}")
        except Exception as e:
            print(f"控制错误: {e}")

# 关闭应用程序
simulation_app.close()