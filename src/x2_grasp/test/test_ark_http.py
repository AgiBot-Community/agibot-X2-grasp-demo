import json

import pytest

from x2_grasp.ark_http import ArkHttpClient, completion_content


class FakeResponse:
    def __init__(self, payload: dict) -> None:
        self._body = json.dumps(payload).encode("utf-8")

    def __enter__(self):
        return self

    def __exit__(self, *_args) -> None:
        return None

    def read(self, size: int = -1) -> bytes:
        return self._body if size < 0 else self._body[:size]


def test_client_uses_openai_compatible_chat_endpoint(monkeypatch) -> None:
    captured = {}

    def fake_urlopen(request, timeout):
        captured["url"] = request.full_url
        captured["authorization"] = request.get_header("Authorization")
        captured["body"] = json.loads(request.data.decode("utf-8"))
        captured["timeout"] = timeout
        return FakeResponse(
            {"choices": [{"message": {"content": "<bbox>1 2 3 4</bbox>"}}]}
        )

    monkeypatch.setattr("x2_grasp.ark_http.urlopen", fake_urlopen)
    client = ArkHttpClient("secret", "https://example.test/api/v3/", 12.5)
    response = client.create_chat_completion(
        model="vision-model",
        messages=[{"role": "user", "content": "locate"}],
    )

    assert captured == {
        "url": "https://example.test/api/v3/chat/completions",
        "authorization": "Bearer secret",
        "body": {
            "model": "vision-model",
            "messages": [{"role": "user", "content": "locate"}],
        },
        "timeout": 12.5,
    }
    assert completion_content(response) == "<bbox>1 2 3 4</bbox>"


def test_completion_content_rejects_malformed_response() -> None:
    with pytest.raises(RuntimeError, match="no completion content"):
        completion_content({"choices": []})


@pytest.mark.parametrize(
    ("base_url", "timeout"),
    [
        ("http://example.test/api", 1.0),
        ("not-a-url", 1.0),
        ("https://example.test/api", 0.0),
        ("https://example.test/api", float("nan")),
    ],
)
def test_client_rejects_unsafe_endpoint_or_timeout(base_url, timeout) -> None:
    with pytest.raises(ValueError):
        ArkHttpClient("secret", base_url, timeout)


def test_client_rejects_oversized_response(monkeypatch) -> None:
    monkeypatch.setattr(
        "x2_grasp.ark_http.urlopen",
        lambda *_args, **_kwargs: FakeResponse({"payload": "x" * 100}),
    )
    client = ArkHttpClient(
        "secret", "https://example.test/api", 1.0, max_response_bytes=20
    )

    with pytest.raises(RuntimeError, match="size limit"):
        client.create_chat_completion(model="model", messages=[])
