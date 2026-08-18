from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

try:
    import builtin_interfaces.msg  # noqa: F401
except ImportError:
    builtin_package = ModuleType("builtin_interfaces")
    builtin_messages = ModuleType("builtin_interfaces.msg")

    class _Time:
        def __init__(self, sec: int = 0, nanosec: int = 0) -> None:
            self.sec = sec
            self.nanosec = nanosec

    builtin_messages.Time = _Time
    builtin_package.msg = builtin_messages
    sys.modules["builtin_interfaces"] = builtin_package
    sys.modules["builtin_interfaces.msg"] = builtin_messages


try:
    import x2_grasp.msg as _messages
except ImportError:
    import x2_grasp as package

    _messages = ModuleType("x2_grasp.msg")
    package.msg = _messages
    sys.modules["x2_grasp.msg"] = _messages


class _TypedMessage:
    def __init__(self) -> None:
        self.created_at = SimpleNamespace(sec=0, nanosec=0)
        self.image_stamp = SimpleNamespace(sec=0, nanosec=0)
        self.stamp = SimpleNamespace(sec=0, nanosec=0)
        self.request_id = ""
        self.target = ""
        self.target_zh = ""
        self.status = ""
        self.error = ""
        self.success = False
        self.frame_id = ""
        self.image_width = 0
        self.image_height = 0
        self.box_count = 0
        self.bbox_selection = ""
        self.annotated_image_path = ""
        self.latency_ms = 0.0
        self.source = ""
        self.stage = ""
        self.detail = ""
        self.detected_ids = []
        self.sample_count = 0
        self.required_samples = 0
        self.spread_m = 0.0
        self.reprojection_error_px = 0.0


class _PerceptionStatus(_TypedMessage):
    def __init__(self) -> None:
        super().__init__()
        self.target = SimpleNamespace(x=0.0, y=0.0, z=0.0)


for _name, _type in {
    "GroundingCommand": _TypedMessage,
    "GroundingResult": _TypedMessage,
    "PerceptionStatus": _PerceptionStatus,
}.items():
    if not hasattr(_messages, _name):
        setattr(_messages, _name, _type)
