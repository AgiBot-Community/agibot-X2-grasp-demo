"""Threaded ROS 2 Action bridge for the synchronous grasp orchestrator."""

from __future__ import annotations

from dataclasses import dataclass, field
from queue import Empty, Queue
import threading

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from x2_grasp.action import Grasp

@dataclass
class GraspWorkItem:
    goal_handle: object
    request_id: str
    target: str
    finished: threading.Event = field(default_factory=threading.Event)
    cancel_requested: threading.Event = field(default_factory=threading.Event)
    success: bool = False
    canceled: bool = False
    error: str = ""

    def publish_feedback(self, stage: str, detail: str = "") -> None:
        feedback = Grasp.Feedback()
        feedback.stage = stage
        feedback.detail = detail
        self.goal_handle.publish_feedback(feedback)

    def finish(
        self, *, success: bool, error: str = "", canceled: bool = False
    ) -> None:
        self.success = success
        self.error = error
        self.canceled = canceled
        self.finished.set()

    def is_cancel_requested(self) -> bool:
        if self.goal_handle.is_cancel_requested:
            self.cancel_requested.set()
        return self.cancel_requested.is_set()


class GraspActionBridge:
    """Run Action callbacks separately and hand accepted goals to the main loop."""

    def __init__(self, rclpy_module, action_name: str, target_catalog) -> None:
        self._rclpy = rclpy_module
        self._target_catalog = target_catalog
        self._queue: Queue[GraspWorkItem] = Queue(maxsize=1)
        self._reservation_lock = threading.Lock()
        self._goal_reserved = False
        self._stopping = threading.Event()
        self._current: GraspWorkItem | None = None

        self.node = rclpy_module.create_node(
            "x2_grasp_action_server", use_global_arguments=False
        )
        callback_group = ReentrantCallbackGroup()
        self._server = ActionServer(
            self.node,
            Grasp,
            action_name,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=callback_group,
        )
        self._executor = MultiThreadedExecutor(num_threads=2)
        self._executor.add_node(self.node)
        self._thread = threading.Thread(
            target=self._executor.spin,
            name="x2-grasp-action-executor",
            daemon=True,
        )
        self._thread.start()

    def _goal_callback(self, goal_request) -> GoalResponse:
        target = self._target_catalog.normalize(goal_request.target)
        if target is None or self._stopping.is_set():
            return GoalResponse.REJECT
        with self._reservation_lock:
            if self._goal_reserved:
                return GoalResponse.REJECT
            self._goal_reserved = True
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        current = self._current
        if current is None or current.finished.is_set():
            return CancelResponse.REJECT
        current.cancel_requested.set()
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle):
        target = self._target_catalog.normalize(goal_handle.request.target)
        if target is None:
            goal_handle.abort()
            return self._result(False, goal_handle.request.target, "unsupported target")

        work = GraspWorkItem(
            goal_handle=goal_handle,
            request_id=bytes(goal_handle.goal_id.uuid).hex(),
            target=target,
        )
        self._current = work
        self._queue.put(work)
        while not work.finished.wait(0.1):
            if goal_handle.is_cancel_requested:
                work.cancel_requested.set()
            if self._stopping.is_set():
                work.finish(success=False, error="action server is stopping", canceled=True)

        try:
            if work.canceled:
                goal_handle.canceled()
            elif work.success:
                goal_handle.succeed()
            else:
                goal_handle.abort()
            return self._result(work.success, target, work.error)
        finally:
            self._current = None
            with self._reservation_lock:
                self._goal_reserved = False

    @staticmethod
    def _result(success: bool, target: str, error: str):
        result = Grasp.Result()
        result.success = success
        result.target = target
        result.error = error
        return result

    def next_goal(self, timeout: float = 0.1) -> GraspWorkItem | None:
        try:
            return self._queue.get(timeout=timeout)
        except Empty:
            return None

    def goal_done(self) -> None:
        self._queue.task_done()

    def shutdown(self) -> None:
        self._stopping.set()
        current = self._current
        if current is not None and not current.finished.is_set():
            current.finish(success=False, error="action server stopped", canceled=True)
        # Let execute callbacks finish while their goal handles and the server
        # are still valid, then release the ROS entities.
        self._executor.shutdown(timeout_sec=2.0)
        self._thread.join(timeout=2.0)
        self._server.destroy()
        self.node.destroy_node()
