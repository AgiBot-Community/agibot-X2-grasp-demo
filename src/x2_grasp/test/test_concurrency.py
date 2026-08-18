from pathlib import Path
from queue import Queue
import sys

import pytest


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from x2_common.concurrency import LatestValue, put_latest  # noqa: E402


def test_latest_value_consumes_and_expires_values() -> None:
    now = [10.0]
    latest = LatestValue[str](clock=lambda: now[0])
    latest.set("first")
    now[0] = 11.0

    assert latest.peek(max_age_seconds=2.0) == "first"
    assert latest.take(max_age_seconds=2.0) == "first"
    assert latest.take() is None

    latest.set("expired")
    now[0] = 20.0
    assert latest.take(max_age_seconds=2.0) is None


def test_latest_value_rejects_negative_age() -> None:
    latest = LatestValue[str]()
    latest.set("value")

    with pytest.raises(ValueError):
        latest.peek(max_age_seconds=-1.0)


def test_put_latest_drops_oldest_without_blocking() -> None:
    items: Queue[str] = Queue(maxsize=2)
    items.put_nowait("one")
    items.put_nowait("two")

    dropped = put_latest(items, "three")

    assert dropped == "one"
    assert items.get_nowait() == "two"
    assert items.get_nowait() == "three"
