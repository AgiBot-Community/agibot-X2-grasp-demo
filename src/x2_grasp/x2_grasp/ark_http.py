"""Small standard-library client for Ark's OpenAI-compatible endpoint."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.parse import urlparse
from urllib.request import Request, urlopen


DEFAULT_MAX_RESPONSE_BYTES = 2 * 1024 * 1024


def _error_message(body: bytes, fallback: str) -> str:
    try:
        payload = json.loads(body.decode("utf-8"))
        error = payload.get("error", {})
        if isinstance(error, dict) and isinstance(error.get("message"), str):
            return error["message"]
    except (UnicodeDecodeError, json.JSONDecodeError, AttributeError):
        pass
    return fallback


@dataclass(frozen=True)
class ArkHttpClient:
    api_key: str
    base_url: str
    timeout: float
    max_response_bytes: int = DEFAULT_MAX_RESPONSE_BYTES

    def __post_init__(self) -> None:
        parsed = urlparse(self.base_url)
        if parsed.scheme != "https" or not parsed.hostname:
            raise ValueError("Ark base_url must be an absolute HTTPS URL")
        if not self.api_key.strip():
            raise ValueError("Ark api_key must not be empty")
        if not math.isfinite(self.timeout) or self.timeout <= 0.0:
            raise ValueError("Ark timeout must be a positive finite value")
        if self.max_response_bytes < 1:
            raise ValueError("Ark max_response_bytes must be positive")

    def create_chat_completion(
        self,
        *,
        model: str,
        messages: list[dict[str, Any]],
    ) -> dict[str, Any]:
        endpoint = f"{self.base_url.rstrip('/')}/chat/completions"
        body = json.dumps(
            {"model": model, "messages": messages},
            ensure_ascii=False,
            separators=(",", ":"),
        ).encode("utf-8")
        request = Request(
            endpoint,
            data=body,
            headers={
                "Authorization": f"Bearer {self.api_key}",
                "Content-Type": "application/json",
                "Accept": "application/json",
            },
            method="POST",
        )
        try:
            with urlopen(request, timeout=self.timeout) as response:
                response_body = response.read(self.max_response_bytes + 1)
        except HTTPError as exc:
            error_body = exc.read(self.max_response_bytes + 1)
            detail = _error_message(error_body, exc.reason or "HTTP error")
            raise RuntimeError(f"Ark API HTTP {exc.code}: {detail}") from exc
        except URLError as exc:
            raise RuntimeError(f"Ark API network error: {exc.reason}") from exc

        if len(response_body) > self.max_response_bytes:
            raise RuntimeError("Ark API response exceeds the configured size limit")
        try:
            payload = json.loads(response_body.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise RuntimeError("Ark API returned invalid JSON") from exc
        if not isinstance(payload, dict):
            raise RuntimeError("Ark API returned a non-object JSON response")
        return payload


def completion_content(response: dict[str, Any]) -> Any:
    try:
        choices = response["choices"]
        return choices[0]["message"]["content"]
    except (KeyError, IndexError, TypeError) as exc:
        raise RuntimeError("Ark API response has no completion content") from exc
