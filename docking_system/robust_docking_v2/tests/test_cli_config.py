"""Field commands must never fall back to test geometry."""
import io
import unittest
from unittest.mock import patch
from docking.__main__ import main


class ConfigSelectionTests(unittest.TestCase):
    def test_policy_generation_requires_an_explicit_config(self):
        with patch('sys.argv', ['docking', 'synthesize']), patch('sys.stderr', new_callable=io.StringIO), self.assertRaises(SystemExit) as exc:
            main()
        self.assertEqual(exc.exception.code, 2)

    def test_test_only_and_retired_commands_are_unavailable(self):
        for command in ('simulate', 'make-variants'):
            with self.subTest(command=command), patch('sys.argv', ['docking', command]), patch('sys.stderr', new_callable=io.StringIO), self.assertRaises(SystemExit) as exc:
                main()
            self.assertEqual(exc.exception.code, 2)
