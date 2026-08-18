"""Validated, configuration-driven grasp target catalog."""

from __future__ import annotations

from dataclasses import dataclass
import math
import re


DEFAULT_TARGET_NAMES = ["cup", "bread", "bottle"]
DEFAULT_TARGET_DESCRIPTIONS = [
    "一次性纸杯",
    "长条的玉米面包，半透明平口塑料袋用扎丝封口的长条吐司面包",
    "长条药瓶",
]
DEFAULT_TARGET_ALIASES = ["paper_cup=cup", "medicine_bottle=bottle"]
DEFAULT_TARGET_GRIP_POSITIONS = [0.10, 0.10, 0.10]
DEFAULT_TARGET_PCM_PATHS = ["cup.pcm", "bread.pcm", "bottle.pcm"]

TARGET_NAME_PATTERN = re.compile(r"^[a-z][a-z0-9_-]{0,63}$")


@dataclass(frozen=True)
class TargetSpec:
    name: str
    description: str
    grip_close_position: float
    pcm_path: str


class TargetCatalog:
    def __init__(self, specs: dict[str, TargetSpec], aliases: dict[str, str]):
        self.specs = specs
        self.aliases = aliases

    @property
    def names(self) -> tuple[str, ...]:
        return tuple(self.specs)

    @property
    def descriptions(self) -> dict[str, str]:
        return {name: spec.description for name, spec in self.specs.items()}

    def normalize(self, value: str) -> str | None:
        normalized = value.strip().lower()
        normalized = self.aliases.get(normalized, normalized)
        return normalized if normalized in self.specs else None

    def spec(self, target: str) -> TargetSpec:
        return self.specs[target]


def build_target_catalog(
    names,
    descriptions,
    aliases,
    grip_close_positions=None,
    pcm_paths=None,
) -> TargetCatalog:
    names = [str(value).strip().lower() for value in names]
    descriptions = [str(value).strip() for value in descriptions]
    grip_close_positions = list(
        [0.10] * len(names) if grip_close_positions is None else grip_close_positions
    )
    pcm_paths = list(
        [f"{name}.pcm" for name in names] if pcm_paths is None else pcm_paths
    )

    if not names:
        raise ValueError("target_names must contain at least one target")
    expected = len(names)
    for parameter_name, values in (
        ("target_descriptions", descriptions),
        ("target_grip_close_positions", grip_close_positions),
        ("target_pcm_paths", pcm_paths),
    ):
        if len(values) != expected:
            raise ValueError(
                f"{parameter_name} must have the same length as target_names"
            )
    if len(set(names)) != expected:
        raise ValueError("target_names must not contain duplicates")

    specs = {}
    for name, description, grip_position, pcm_path in zip(
        names,
        descriptions,
        grip_close_positions,
        pcm_paths,
    ):
        if not TARGET_NAME_PATTERN.fullmatch(name):
            raise ValueError(f"invalid target name: {name!r}")
        if not description:
            raise ValueError(f"target description must not be empty: {name}")
        grip_position = float(grip_position)
        if not math.isfinite(grip_position) or not 0.0 <= grip_position <= 1.0:
            raise ValueError(
                f"grip close position for {name} must be finite and in [0, 1]"
            )
        specs[name] = TargetSpec(
            name,
            description,
            grip_position,
            str(pcm_path).strip(),
        )

    alias_map = {}
    for item in aliases:
        alias, separator, canonical = str(item).partition("=")
        alias = alias.strip().lower()
        canonical = canonical.strip().lower()
        if not separator or not TARGET_NAME_PATTERN.fullmatch(alias):
            raise ValueError(f"target alias must use alias=target syntax: {item!r}")
        if canonical not in specs:
            raise ValueError(f"target alias references an unknown target: {item!r}")
        if alias in specs or alias in alias_map:
            raise ValueError(f"duplicate or conflicting target alias: {alias}")
        alias_map[alias] = canonical
    return TargetCatalog(specs, alias_map)


def default_target_catalog() -> TargetCatalog:
    return build_target_catalog(
        DEFAULT_TARGET_NAMES,
        DEFAULT_TARGET_DESCRIPTIONS,
        DEFAULT_TARGET_ALIASES,
        DEFAULT_TARGET_GRIP_POSITIONS,
        DEFAULT_TARGET_PCM_PATHS,
    )
