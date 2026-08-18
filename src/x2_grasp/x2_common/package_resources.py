"""Resolve ROS package resources in source and install layouts."""

from __future__ import annotations

from pathlib import Path


def package_directory(
    package_name: str,
    *,
    source_root: str | Path | None = None,
) -> Path:
    """Return a source package root when available, otherwise its share path."""
    if not package_name.strip():
        raise ValueError("package_name must not be empty")
    if source_root is not None:
        source = Path(source_root).expanduser().resolve()
        if (source / "package.xml").is_file():
            return source
    try:
        from ament_index_python.packages import get_package_share_directory
    except ImportError as error:
        raise RuntimeError(
            f"cannot locate ROS package {package_name!r}; source the workspace first"
        ) from error
    try:
        return Path(get_package_share_directory(package_name)).resolve()
    except LookupError as error:
        raise RuntimeError(f"ROS package not found: {package_name}") from error


def package_file(
    package_name: str,
    relative_path: str | Path,
    *,
    source_root: str | Path | None = None,
    require_exists: bool = False,
) -> Path:
    """Resolve an absolute path or a path relative to a ROS package share."""
    path = Path(relative_path).expanduser()
    if not path.is_absolute():
        path = package_directory(package_name, source_root=source_root) / path
    path = path.resolve()
    if require_exists and not path.is_file():
        raise FileNotFoundError(f"package resource does not exist: {path}")
    return path
