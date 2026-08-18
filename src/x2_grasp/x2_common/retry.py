"""Interruptible bounded retry and cleanup helpers."""

from __future__ import annotations

from collections.abc import Callable
import threading
from typing import TypeVar


T = TypeVar("T")


class RetryExhaustedError(RuntimeError):
    """Raised after a bounded operation has used all of its attempts."""


def run_bounded_cleanup(
    cleanup: Callable[[], None],
    *,
    timeout_seconds: float,
) -> bool:
    """Run best-effort cleanup without letting a stuck SDK block retry forever."""
    if timeout_seconds < 0:
        raise ValueError("timeout_seconds cannot be negative")
    finished = threading.Event()

    def run() -> None:
        try:
            cleanup()
        except Exception:
            pass
        finally:
            finished.set()

    thread = threading.Thread(
        target=run,
        name="x2-common-bounded-cleanup",
        daemon=True,
    )
    thread.start()
    return finished.wait(timeout_seconds)


def run_with_retry(
    operation: Callable[[int], T],
    stop_requested: threading.Event,
    *,
    max_attempts: int,
    initial_delay_seconds: float,
    max_delay_seconds: float,
    on_retry: Callable[[int, float, Exception], None] | None = None,
    wait_for_stop: Callable[[float], bool] | None = None,
) -> T | None:
    """Run an operation with bounded exponential backoff.

    ``operation`` receives a one-based attempt number. Returning means success;
    raising requests another attempt unless the retry budget is exhausted.
    """
    if max_attempts < 1:
        raise ValueError("max_attempts must be at least 1")
    if initial_delay_seconds < 0 or max_delay_seconds < 0:
        raise ValueError("retry delays cannot be negative")

    wait = wait_for_stop or stop_requested.wait
    last_error: Exception | None = None
    for attempt in range(1, max_attempts + 1):
        if stop_requested.is_set():
            return None
        try:
            return operation(attempt)
        except Exception as exc:
            last_error = exc
            if stop_requested.is_set():
                return None
            if attempt >= max_attempts:
                break
            delay = min(
                max_delay_seconds,
                initial_delay_seconds * (2 ** (attempt - 1)),
            )
            if on_retry is not None:
                on_retry(attempt, delay, exc)
            if wait(delay):
                return None

    raise RetryExhaustedError(
        f"operation failed after {max_attempts} attempts: {last_error}"
    ) from last_error
