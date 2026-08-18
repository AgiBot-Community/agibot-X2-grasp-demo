from pathlib import Path
import shutil
import sys
import unittest


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from x2_common.credentials import load_secret_file, resolve_secret  # noqa: E402


class CredentialTests(unittest.TestCase):
    def setUp(self) -> None:
        self.root = PROJECT_ROOT / "test/.credential_test"
        shutil.rmtree(self.root, ignore_errors=True)
        self.root.mkdir()

    def tearDown(self) -> None:
        shutil.rmtree(self.root, ignore_errors=True)

    def test_loads_nested_json_and_simple_key_value_files(self) -> None:
        json_path = self.root / "secret.json"
        json_path.write_text('{"provider":{"api_key":"json-key"}}', encoding="utf-8")
        yaml_path = self.root / "secret.yaml"
        yaml_path.write_text('api_key: "yaml-key"\n', encoding="utf-8")
        env_path = self.root / "key.txt"
        env_path.write_text("DASHSCOPE_API_KEY=env-file-key\n", encoding="utf-8")

        self.assertEqual(load_secret_file(json_path, keys=("api_key",)), "json-key")
        self.assertEqual(load_secret_file(yaml_path, keys=("api_key",)), "yaml-key")
        self.assertEqual(
            load_secret_file(env_path, keys=("dashscope_api_key",)),
            "env-file-key",
        )

    def test_environment_has_precedence_without_exposing_value(self) -> None:
        path = self.root / "secret.yaml"
        path.write_text("api_key: file-key\n", encoding="utf-8")

        resolved = resolve_secret(
            environment_variable="TEST_KEY",
            configured_value="configured-key",
            file_path=path,
            keys=("api_key",),
            environment={"TEST_KEY": "environment-key"},
        )

        self.assertEqual(resolved, "environment-key")

    def test_missing_or_invalid_file_returns_empty_value(self) -> None:
        invalid = self.root / "invalid.yaml"
        invalid.write_text("api_key: 'unterminated\n", encoding="utf-8")

        self.assertEqual(load_secret_file(self.root / "missing"), "")
        self.assertEqual(load_secret_file(invalid, keys=("api_key",)), "")


if __name__ == "__main__":
    unittest.main()
