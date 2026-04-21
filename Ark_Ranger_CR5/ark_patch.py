# ark_patch.py
import os
import sys
import types

ENABLE_COMPAT = str(os.getenv("ARK_ENABLE_COMPAT", "")).strip().lower() in ("1", "true", "yes")

if ENABLE_COMPAT:
    # 1. Inject legacy paths (compat mode only)
    current_dir = os.path.dirname(os.path.abspath(__file__))
    paths = [
        os.path.join(current_dir, "ark_framework"),
        os.path.join(current_dir, "ark_types"),
        os.path.join(current_dir, "src"),
        current_dir,
    ]
    for p in paths:
        if os.path.exists(p) and p not in sys.path:
            sys.path.insert(0, p)

    # 2. Mock arktypes for legacy LCM runtime
    class MockLCMType:
        def __init__(self, **kwargs):
            self.__dict__.update(kwargs)

        def encode(self):
            return b""

        @staticmethod
        def decode(data):
            return MockLCMType()

    class ArkTypesMockModule(types.ModuleType):
        def __getattr__(self, name):
            if name.startswith("__"):
                raise AttributeError(name)
            mock_cls = type(name, (MockLCMType,), {})
            setattr(self, name, mock_cls)
            return mock_cls

    if "arktypes" not in sys.modules:
        mod = ArkTypesMockModule("arktypes")
        mod.__file__ = __file__
        sys.modules["arktypes"] = mod

    # 3. Import base node and handle compat behavior
    try:
        from ark.client.comm_infrastructure.base_node import BaseNode
    except ImportError:
        class BaseNode:  # type: ignore
            def __init__(self, *args, **kwargs):
                pass

    class Node(BaseNode):
        def __init__(self, node_name, **kwargs):
            try:
                super().__init__(node_name)
            except TypeError:
                try:
                    super().__init__(node_name, **kwargs)
                except Exception:
                    super().__init__()
            print(f"[Patch] Node '{node_name}' initialized (compat mode)")

        def create_subscription(self, type, topic, callback):
            # Mock subscription handler for legacy internal routing
            self._callback = callback

else:
    # Compatibility disabled: provide lightweight stub only.
    class BaseNode:  # type: ignore
        def __init__(self, *args, **kwargs):
            pass

    class Node(BaseNode):
        def __init__(self, node_name, **kwargs):
            super().__init__()

        def create_subscription(self, type, topic, callback):
            self._callback = callback

