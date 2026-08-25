from threading import Event, Lock

import pytest

pytest.importorskip("rclpy")

from x2_grasp.grounding_node import GroundingNode
from x2_grasp.msg import GroundingCommand
from x2_grasp.target_catalog import default_target_catalog


def test_resolve_relative_api_key_from_package_config() -> None:
    resolved = GroundingNode._resolve_api_key_file("api_key.yaml")

    assert resolved.endswith("x2_grasp/config/api_key.yaml")


def test_target_command_preserves_request_id() -> None:
    node = object.__new__(GroundingNode)
    node._stop_requested = Event()
    node._image_lock = Lock()
    node._latest_image = None
    node._target_catalog = default_target_catalog()
    published = []
    node._publish_error = lambda request_id, target, error: published.append(
        (request_id, target, error))

    message = GroundingCommand()
    message.target = "cup"
    message.request_id = "attempt-2"
    node._on_target(message)

    assert published[0][0] == "attempt-2"
    assert published[0][1] == "cup"
