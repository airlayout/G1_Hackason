"""Offline tests for bundle integrity and rejected model bytes."""
import hashlib
import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch


def load(name):
    spec = importlib.util.spec_from_file_location(name, Path(__file__).parent / (name + '.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class IntegrityTests(unittest.TestCase):
    def bundle(self, root):
        module = load('rebuild_runtime')
        for name in ('code', 'deps'):
            directory = root / name
            directory.mkdir()
            (directory / 'asset').write_bytes(b'recorded asset')
            (directory / 'MANIFEST.sha256').write_text(module.hashes(directory))
        (root / 'BUNDLE_MANIFEST.sha256').write_text(module.hashes(root))
        return module

    def test_manifest_accepts_bundle_then_rejects_tampered_asset(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            module = self.bundle(root)
            module.verify(root)
            (root / 'code/asset').write_bytes(b'changed asset')
            with self.assertRaisesRegex(ValueError, 'Hash mismatch'):
                module.verify(root)

    def test_manifest_refuses_path_escape(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            module = self.bundle(root)
            (root / 'code/MANIFEST.sha256').write_text('0' * 64 + '  ../../outside\n')
            with self.assertRaisesRegex(ValueError, 'escapes bundle'):
                module.verify(root)

    def test_empty_source_only_deps_manifest_is_supported(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            module = self.bundle(root)
            (root / 'deps/asset').unlink()
            (root / 'deps/MANIFEST.sha256').write_text('\n')
            (root / 'BUNDLE_MANIFEST.sha256').write_text(module.hashes(root))
            module.verify(root)

    def test_yolo_wrong_existing_hash_is_rejected_without_replacing_file(self):
        module = load('fetch_yolo')
        with tempfile.TemporaryDirectory() as directory:
            dest = Path(directory) / 'model.pt'
            dest.write_bytes(b'bad')
            with patch.object(module, 'DEST', dest):
                with self.assertRaises(SystemExit):
                    module.main()
            self.assertEqual(dest.read_bytes(), b'bad')


if __name__ == '__main__':
    unittest.main()
