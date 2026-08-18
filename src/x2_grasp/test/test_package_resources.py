from pathlib import Path
import shutil
import sys
import unittest


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from x2_common.package_resources import package_directory, package_file  # noqa: E402


class PackageResourceTests(unittest.TestCase):
    def setUp(self) -> None:
        self.root = PROJECT_ROOT / "test/.resource_test"
        shutil.rmtree(self.root, ignore_errors=True)
        self.root.mkdir()
        (self.root / "package.xml").write_text("<package />\n", encoding="utf-8")

    def tearDown(self) -> None:
        shutil.rmtree(self.root, ignore_errors=True)

    def test_source_package_is_preferred_before_ament_lookup(self) -> None:
        self.assertEqual(
            package_directory("test_package", source_root=self.root),
            self.root.resolve(),
        )

    def test_relative_and_absolute_resources_resolve_consistently(self) -> None:
        resource = self.root / "config/value.json"
        resource.parent.mkdir()
        resource.write_text("{}\n", encoding="utf-8")

        self.assertEqual(
            package_file(
                "test_package",
                "config/value.json",
                source_root=self.root,
                require_exists=True,
            ),
            resource.resolve(),
        )
        self.assertEqual(
            package_file("ignored", resource, require_exists=True),
            resource.resolve(),
        )


if __name__ == "__main__":
    unittest.main()
