"""Grasp announcement adapter built on the shared PCM player."""

from pathlib import Path

from x2_common import BlockingPcmPlayer, package_file

from .target_catalog import DEFAULT_TARGET_NAMES


class GraspAudioPlayer(BlockingPcmPlayer):
    """Resolve grasp announcement assets and play them through AIMDK."""

    def __init__(self, node, args):
        super().__init__(
            node,
            playback_topic=args.audio_playback_topic,
            package_name=args.audio_pkg_name,
            focus_priority=args.audio_focus_priority,
            chunk_ms=args.audio_chunk_ms,
            focus_response_topic=args.audio_focus_response_topic,
            focus_request_service=args.audio_focus_request_service,
            focus_release_service=args.audio_focus_release_service,
        )
        catalog = getattr(args, "target_catalog", None)
        if catalog is not None:
            configured_paths = {
                target: spec.pcm_path for target, spec in catalog.specs.items()
            }
        else:
            configured_paths = {
                target: getattr(args, f"{target}_pcm_path")
                for target in DEFAULT_TARGET_NAMES
            }
        self.pcm_paths = {
            target: self._resolve_pcm_path(path) if path else None
            for target, path in configured_paths.items()
        }
        self.default_pcm_path = self._resolve_pcm_path(
            getattr(args, "default_pcm_path", "grasp_complete.pcm")
        )

    @staticmethod
    def _resolve_pcm_path(path_value):
        return package_file(
            "x2_grasp",
            Path("audio") / Path(path_value).expanduser(),
            source_root=Path(__file__).resolve().parents[1],
        )

    def play(self, rclpy_mod, target):
        primary_path = self.pcm_paths[target]
        paths = [path for path in (primary_path, self.default_pcm_path) if path]
        paths = list(dict.fromkeys(paths))
        pcm = None
        errors = []
        for path in paths:
            try:
                pcm = path.read_bytes()
                if path == self.default_pcm_path and path != primary_path:
                    self.node.get_logger().warning(
                        f"using default completion audio for target={target}: {path}"
                    )
                break
            except OSError as error:
                errors.append(f"{path}: {error}")
        if pcm is None:
            raise RuntimeError(
                f"cannot read completion audio for {target}: {'; '.join(errors)}"
            )
        return super().play(rclpy_mod, pcm, stream_name=target)
