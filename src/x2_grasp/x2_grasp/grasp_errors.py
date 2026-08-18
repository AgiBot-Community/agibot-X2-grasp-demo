"""Shared grasp workflow exceptions."""


class GraspCancelled(RuntimeError):
    """Raised when an accepted grasp goal is canceled."""
