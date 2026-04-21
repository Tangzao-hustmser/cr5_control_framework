
# 极限安全具身智能体项目

## 当前进展

1. 已配置好 USD 与 URDF 文件。
2. 正在尝试移植开源代码：
   - 原代码基于 Isaac Sim 的 RMPFlow 类，用于计算机械臂逆解。
   - 直接应用于 CR5时，误差会在0.2-0.3左右，差别过大，难以接受。
   - 同时，机械臂碰撞未成功传入RMPFlow算法，导致仍然会有穿模（可能RMPFlow类未被使用？）
3. 尝试根据开源代码写了一份非追踪式的仿真历程（arm_swing_example.py）
   - 基于 Isaac Sim 的 RMPFlow 类，用于计算机械臂逆解。
   - 直接应用于 CR5时，误差会在0.5左右，差别较大，但由于目标点是函数定义，一直在变化。
   - 场景：机械臂按正弦方式摆臂
4. 写好新的框架（进行中）：
   - 借助开源框架ark
   - 使用方法：
     - vscode启动终端A输入：
       - cd ..\Ark_Ranger_CR5 #进入项目目录
       - ..\python.bat run_registry.py
     - 然后先下载vscode的issacsim插件
     - 在运行与调试界面选项卡，利用 [python:current file] 运行调试 run_simulation.py;（自动调出终端B）
     - 再建新的终端C，输入下列命令用来实时监控机械臂实时状态
       - python scripts/watch_live_status.py
     - 终端B中，等待出现 `r3-json>`(一般要按回车)，然后终端一条“单行 JSON”，回车输入
     - 机械臂会根据所给目标位置计算机械臂位姿，机械臂摆动到对应位置。
     - 一条 `pose` 命令示例（可直接粘贴）
       - （机械臂末端位置及姿态）
     ```json
     {"command_type":"pose","payload":{"frame":"world","position_m":[0.45,0.15,0.8],"orientation_wxyz":[1,0,0,0]}}
     ```
       - （关节角度）
      ```json
      arm 0 -30 45 0 0 0
      ```
      ```json
      arm_rad 0 -0.52 0.79 0 0 0
      ```
      - 也可查看项目文件中..\Ark_Ranger_CR5\docs\simulation_guide_zh.md
5. 计划
   - 正在测试框架稳定性与尝试接入大模型
   - 正在加入ROS2控制

