"""Credential loading helpers that never expose secret values."""

from __future__ import annotations

import ast
import json
import os
from pathlib import Path
import re
from typing import Any, Iterable, Mapping


DEFAULT_SECRET_KEYS = ("api_key", "dashscope_api_key")


def _nested_string(payload: Any, keys: set[str]) -> str:
    pending = [payload]
    while pending:
        current = pending.pop()
        if isinstance(current, dict):
            for name, value in current.items():
                if str(name).lower() in keys and isinstance(value, str):
                    if value.strip():
                        return value.strip()
                pending.append(value)
        elif isinstance(current, list):
            pending.extend(current)
    return ""


def load_secret_file(
    path_value: str | Path | None,
    *,
    keys: Iterable[str] = DEFAULT_SECRET_KEYS,
) -> str:
    """Read a secret from JSON or a simple ``key: value``/``key=value`` file."""
    if not path_value:
        return ""
    path = Path(path_value).expanduser()
    if not path.is_file():
        return ""
    try:
        text = path.read_text(encoding="utf-8-sig")
    except OSError:
        return ""

    normalized_keys = {str(key).strip().lower() for key in keys}
    try:
        value = _nested_string(json.loads(text), normalized_keys)
        if value:
            return value
    except json.JSONDecodeError:
        pass

    key_pattern = "|".join(re.escape(key) for key in sorted(normalized_keys))
    pattern = re.compile(rf"^\s*(?:{key_pattern})\s*[:=]\s*(.*?)\s*$", re.I)
    for line in text.splitlines():
        match = pattern.match(line)
        if not match:
            continue
        value = match.group(1).split(" #", 1)[0].strip()
        quote_chars = {'"', "'"}
        if value[:1] in quote_chars or value[-1:] in quote_chars:
            if len(value) < 2 or value[0] != value[-1]:
                return ""
            try:
                value = ast.literal_eval(value)
            except (SyntaxError, ValueError):
                return ""
        return value.strip() if isinstance(value, str) else ""
    return ""


def resolve_secret(
    *,
    environment_variable: str = "",
    configured_value: str = "",
    file_path: str | Path | None = None,
    keys: Iterable[str] = DEFAULT_SECRET_KEYS,
    environment: Mapping[str, str] | None = None,
) -> str:
    """Resolve a secret using environment, explicit config, then file order."""
    values = os.environ if environment is None else environment
    if environment_variable:
        environment_value = values.get(environment_variable, "").strip()
        if environment_value:
            return environment_value
    configured = str(configured_value).strip()
    if configured:
        return configured
    return load_secret_file(file_path, keys=keys)
