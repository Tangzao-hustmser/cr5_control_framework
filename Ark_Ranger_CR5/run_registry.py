# run_registry.py
import os
import sys
import types

ENABLE_COMPAT = str(os.getenv("ARK_ENABLE_COMPAT", "")).strip().lower() in ("1", "true", "yes")

if not ENABLE_COMPAT:
    print("[Registry] Compatibility mode disabled. Set ARK_ENABLE_COMPAT=1 to run legacy registry.")
    raise SystemExit(0)

# 1. Inject legacy paths
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(current_dir, "ark_framework"))
sys.path.insert(0, os.path.join(current_dir, "ark_types"))

# 2. Mock arktypes dependency before loading framework
if "arktypes" not in sys.modules:
    mock_arktypes = types.ModuleType("arktypes")
    sys.modules["arktypes"] = mock_arktypes

import arktypes

# Provide minimal classes required by Registry
for name in ["flag_t", "network_info_t", "node_info_t"]:
    if not hasattr(arktypes, name):
        mock_cls = type(
            name,
            (object,),
            {
                "encode": lambda self: b"",
                "decode": staticmethod(lambda data: mock_cls()),
            },
        )
        setattr(arktypes, name, mock_cls)

print("[Registry Patch] Legacy dependencies injected. Starting registry...")

# 3. Load and start registry
try:
    from ark.client.comm_infrastructure.registry import Registry

    r = Registry()
    print("--- Ark Registry started (compat mode) ---")
    r.start()

    import time

    while True:
        time.sleep(1)
except Exception as e:
    print(f"[Fatal] Registry start failed: {e}")
    try:
        from ark.client.comm_infrastructure.registry import main

        main()
    except Exception:
        pass

