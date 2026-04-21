# run_registry.py
import sys
import os
import types

# 1. 注入源码路径
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(current_dir, "ark_framework"))
sys.path.insert(0, os.path.join(current_dir, "ark_types"))

# 2. 核心：在加载框架前，先伪造 arktypes 依赖
if 'arktypes' not in sys.modules:
    mock_arktypes = types.ModuleType('arktypes')
    sys.modules['arktypes'] = mock_arktypes

import arktypes
# 批量伪造 Registry 启动所需的全部类型
for name in ['flag_t', 'network_info_t', 'node_info_t']:
    if not hasattr(arktypes, name):
        mock_cls = type(name, (object,), {
            "encode": lambda self: b"",
            "decode": staticmethod(lambda data: mock_cls())
        })
        setattr(arktypes, name, mock_cls)

print("[Registry Patch] 依赖注入完成，正在启动...")

# 3. 现在再导入真正的 Registry 逻辑
try:
    # 尝试导入并运行
    from ark.client.comm_infrastructure.registry import Registry
    # 模拟命令行启动
    r = Registry()
    print("--- Ark Registry 已成功启动 ---")
    r.start() 
    # 保持运行
    import time
    while True:
        time.sleep(1)
except Exception as e:
    print(f"[Fatal] 启动失败: {e}")
    # 如果上面的 Registry 类没有 start 方法，尝试调用模块的 main (如果有)
    try:
        from ark.client.comm_infrastructure.registry import main
        main()
    except:
        pass