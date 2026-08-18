"""Small concurrency primitives shared by ROS callback-driven packages."""

from __future__ import annotations

from queue import Empty, Full, Queue
from threading import Lock
import time
from typing import Callable, Generic, TypeVar


T = TypeVar("T")


class LatestValue(Generic[T]):
    """Store and optionally consume one timestamped latest value."""

    def __init__(self, *, clock: Callable[[], float] = time.monotonic) -> None:
        self._clock = clock
        self._lock = Lock()
        self._value: T | None = None
        self._received_at: float | None = None

    def set(self, value: T, *, received_at: float | None = None) -> None:
        with self._lock:
            self._value = value
            self._received_at = self._clock() if received_at is None else received_at

    def clear(self) -> None:
        with self._lock:
            self._value = None
            self._received_at = None

    def peek(self, *, max_age_seconds: float | None = None) -> T | None:
        with self._lock:
            return self._get_locked(max_age_seconds=max_age_seconds, consume=False)

    def take(self, *, max_age_seconds: float | None = None) -> T | None:
        with self._lock:
            return self._get_locked(max_age_seconds=max_age_seconds, consume=True)

    def _get_locked(
        self, *, max_age_seconds: float | None, consume: bool
    ) -> T | None:
        if self._value is None or self._received_at is None:
            return None
        if max_age_seconds is not None:
            if max_age_seconds < 0:
                raise ValueError("max_age_seconds cannot be negative")
            if self._clock() - self._received_at > max_age_seconds:
                self._value = None
                self._received_at = None
                return None
        value = self._value
        if consume:
            self._value = None
            self._received_at = None
        return value


def put_latest(
    items: Queue[T],
    item: T,
    *,
    mark_dropped_done: bool = False,
) -> T | None:
    """Insert without blocking, dropping the oldest queued item when full."""
    dropped: T | None = None
    while True:
        try:
            items.put_nowait(item)
            return dropped
        except Full:
            try:
                dropped = items.get_nowait()
            except Empty:
                continue
            if mark_dropped_done:
                items.task_done()
