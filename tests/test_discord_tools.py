#!/usr/bin/env python3
"""Unit tests for scripts/discord_tools.py."""

import json
import os
import sys
import tempfile
import unittest
from io import StringIO
from pathlib import Path
from unittest.mock import MagicMock, patch


sys.modules.setdefault("requests", MagicMock())
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

import discord_tools  # noqa: E402


class TestDiscordToolsConfigResolution(unittest.TestCase):
    def _write_config(self, path, token="TOKEN"):
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w", encoding="utf-8") as handle:
            json.dump({"channels": {"discord": {"token": token}}}, handle)

    def test_explicit_config_wins(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            explicit = os.path.join(tmpdir, "custom.json")
            self._write_config(explicit, token="EXPLICIT")
            self.assertEqual(discord_tools.resolve_config_path(explicit), explicit)

    def test_nanobot_config_fallback(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            nanobot = os.path.join(tmpdir, "nanobot", "config.json")
            standalone = os.path.join(tmpdir, "standalone", "discord_tools.json")
            self._write_config(nanobot, token="NANOBOT")
            with patch.object(discord_tools, "DEFAULT_NANOBOT_CONFIG", nanobot), \
                 patch.object(discord_tools, "DEFAULT_STANDALONE_CONFIG", standalone):
                self.assertEqual(discord_tools.resolve_config_path(None), nanobot)
                loaded = discord_tools.load_config(None)
                self.assertEqual(loaded["channels"]["discord"]["token"], "NANOBOT")

    def test_standalone_config_fallback(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            nanobot = os.path.join(tmpdir, "nanobot", "config.json")
            standalone = os.path.join(tmpdir, "standalone", "discord_tools.json")
            self._write_config(standalone, token="STANDALONE")
            with patch.object(discord_tools, "DEFAULT_NANOBOT_CONFIG", nanobot), \
                 patch.object(discord_tools, "DEFAULT_STANDALONE_CONFIG", standalone):
                self.assertEqual(discord_tools.resolve_config_path(None), standalone)
                loaded = discord_tools.load_config(None)
                self.assertEqual(loaded["channels"]["discord"]["token"], "STANDALONE")

    def test_missing_config_reports_both_locations(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            nanobot = os.path.join(tmpdir, "nanobot", "config.json")
            standalone = os.path.join(tmpdir, "standalone", "discord_tools.json")
            with patch.object(discord_tools, "DEFAULT_NANOBOT_CONFIG", nanobot), \
                 patch.object(discord_tools, "DEFAULT_STANDALONE_CONFIG", standalone), \
                 patch("sys.stderr", new=StringIO()):
                with self.assertRaises(SystemExit) as ctx:
                    discord_tools.resolve_config_path(None)
                self.assertEqual(ctx.exception.code, 1)

    def test_invalid_schema_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            config_path = os.path.join(tmpdir, "discord_tools.json")
            self._write_config(config_path, token="")
            with patch("sys.stderr", new=StringIO()):
                with self.assertRaises(SystemExit) as ctx:
                    discord_tools.load_config(config_path)
                self.assertEqual(ctx.exception.code, 1)