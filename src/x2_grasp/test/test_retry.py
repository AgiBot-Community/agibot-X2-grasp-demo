from __future__ import annotations

from pathlib import Path
import sys
import threading
import unittest


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from x2_common.retry import (  # noqa: E402
    RetryExhaustedError,
    run_bounded_cleanup,
    run_with_retry,
)


class RetryTests(unittest.TestCase):
    def test_blocked_cleanup_cannot_block_retry_loop(self) -> None:
        release = threading.Event()

        completed = run_bounded_cleanup(
            lambda: release.wait(timeout=1.0),
            timeout_seconds=0.01,
        )
        release.set()

        self.assertFalse(completed)

    def test_failures_use_bounded_exponential_backoff_then_succeed(self) -> None:
        attempts: list[int] = []
        delays: list[float] = []
        retry_reports: list[tuple[int, float, str]] = []

        def operation(attempt: int) -> str:
            attempts.append(attempt)
            if attempt < 3:
                raise ConnectionError(f"failure-{attempt}")
            return "connected"

        result = run_with_retry(
            operation,
            threading.Event(),
            max_attempts=5,
            initial_delay_seconds=1.0,
            max_delay_seconds=8.0,
            on_retry=lambda attempt, delay, exc: retry_reports.append(
                (attempt, delay, str(exc))
            ),
            wait_for_stop=lambda delay: delays.append(delay) or False,
        )

        self.assertEqual(result, "connected")
        self.assertEqual(attempts, [1, 2, 3])
        self.assertEqual(delays, [1.0, 2.0])
        self.assertEqual(
            retry_reports,
            [(1, 1.0, "failure-1"), (2, 2.0, "failure-2")],
        )

    def test_retry_budget_is_finite(self) -> None:
        attempts: list[int] = []

        def operation(attempt: int) -> None:
            attempts.append(attempt)
            raise ConnectionError("offline")

        with self.assertRaises(RetryExhaustedError):
            run_with_retry(
                operation,
                threading.Event(),
                max_attempts=3,
                initial_delay_seconds=1.0,
                max_delay_seconds=8.0,
                wait_for_stop=lambda _delay: False,
            )

        self.assertEqual(attempts, [1, 2, 3])

    def test_stop_during_backoff_interrupts_retry(self) -> None:
        stopped = threading.Event()
        attempts: list[int] = []

        def wait_for_stop(_delay: float) -> bool:
            stopped.set()
            return True

        result = run_with_retry(
            lambda attempt: attempts.append(attempt) or (_ for _ in ()).throw(
                ConnectionError("offline")
            ),
            stopped,
            max_attempts=5,
            initial_delay_seconds=1.0,
            max_delay_seconds=8.0,
            wait_for_stop=wait_for_stop,
        )

        self.assertIsNone(result)
        self.assertEqual(attempts, [1])


if __name__ == "__main__":
    unittest.main()
