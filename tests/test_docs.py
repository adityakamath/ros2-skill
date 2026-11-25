#!/usr/bin/env python3
"""Doc drift guards: generated COMMANDS.md is current, documented commands exist, links resolve."""

import os
import re
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

try:
    import argparse
    from ros2_cli import build_parser
    import gen_commands_doc
    _IMPORT_ERROR = None
except Exception as exc:  # no ROS 2 environment
    _IMPORT_ERROR = exc

DOCS = [ROOT / "SKILL.md", ROOT / "README.md", *sorted((ROOT / "references").glob("*.md"))]


def _tree(parser):
    """{command: {subcommand, ...}} for the first two levels of the argparse tree."""
    out = {}
    for action in parser._actions:
        if isinstance(action, argparse._SubParsersAction):
            for name, sub in action.choices.items():
                subs = set()
                for a in sub._actions:
                    if isinstance(a, argparse._SubParsersAction):
                        subs |= set(a.choices)
                out[name] = subs
    return out


@unittest.skipIf(_IMPORT_ERROR, f"ros2_cli not importable: {_IMPORT_ERROR}")
class TestDocs(unittest.TestCase):
    def test_commands_md_is_current(self):
        current = (ROOT / "references" / "COMMANDS.md").read_text(encoding="utf-8")
        self.assertEqual(current, gen_commands_doc.render(),
                         "references/COMMANDS.md is stale: run scripts/gen_commands_doc.py")

    def test_documented_commands_exist(self):
        tree = _tree(build_parser())
        bad = []
        for doc in DOCS:
            if doc.name in ("COMMANDS.md", "CHANGELOG.md"):
                continue
            for lineno, line in enumerate(doc.read_text(encoding="utf-8").splitlines(), 1):
                for m in re.finditer(r"(?:python3\s+\S*ros2_cli\.py|(?:^|[`;(]\s*)CLI)\s+([a-z0-9-]+)(?:\s+([a-z][a-z0-9-]*))?", line):
                    cmd, sub = m.groups()
                    if cmd.startswith("-"):  # global options such as --timeout
                        continue
                    if cmd not in tree:
                        bad.append(f"{doc.name}:{lineno}: unknown command '{cmd}'")
                    elif sub and tree[cmd] and sub not in tree[cmd] and re.fullmatch(r"[a-z-]+", sub):
                        # a lowercase word after a command with subcommands must be one of them
                        if not any(sub == w for w in ("to", "and", "or", "is", "for", "the", "with", "in", "on", "that")):
                            bad.append(f"{doc.name}:{lineno}: '{cmd} {sub}' is not a subcommand")
        self.assertEqual(bad, [])

    def test_every_parser_subcommand_has_a_handler(self):
        """A subcommand that parses but has no DISPATCH entry just prints help (nav2 mission/place did)."""
        import ros2_cli
        missing = []
        for cmd, subs in _tree(build_parser()).items():
            for sub in (subs or {None}):
                if (cmd, sub) not in ros2_cli.DISPATCH:
                    missing.append((cmd, sub))
        self.assertEqual(missing, [])

    def test_relative_links_resolve(self):
        bad = []
        for doc in DOCS:
            for link in re.findall(r"\]\(([^)#]+)(?:#[^)]*)?\)", doc.read_text(encoding="utf-8")):
                if not link.startswith(("http", "mailto")) and not (doc.parent / link).exists():
                    bad.append(f"{doc.name}: {link}")
        self.assertEqual(bad, [])


if __name__ == "__main__":
    unittest.main()
