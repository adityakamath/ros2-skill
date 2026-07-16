#!/usr/bin/env python3
"""Tests for static eval assets under tests/evals/."""

import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
EVALS_DIR = ROOT / "tests" / "evals"
EVALS_FILE = EVALS_DIR / "evals.json"
QUERIES_FILE = EVALS_DIR / "eval_queries.json"

_EVAL_KEYS = {"id", "prompt", "expected_output", "assertions"}
_QUERY_KEYS = {"id", "query", "should_trigger", "note"}


def _load_json(path: Path):
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def test_eval_files_exist():
    assert EVALS_DIR.is_dir(), "tests/evals directory is missing"
    assert EVALS_FILE.is_file(), "tests/evals/evals.json is missing"
    assert QUERIES_FILE.is_file(), "tests/evals/eval_queries.json is missing"


def test_evals_json_schema():
    data = _load_json(EVALS_FILE)

    assert data.get("skill_name") == "ros2-skill"
    assert isinstance(data.get("evals"), list)
    assert len(data["evals"]) >= 5

    seen_ids = set()
    for item in data["evals"]:
        assert set(item.keys()) == _EVAL_KEYS

        assert isinstance(item.get("id"), int)
        assert item["id"] not in seen_ids
        seen_ids.add(item["id"])

        assert isinstance(item.get("prompt"), str) and len(item["prompt"].strip()) >= 12
        assert isinstance(item.get("expected_output"), str) and len(item["expected_output"].strip()) >= 20
        assert isinstance(item.get("assertions"), list) and len(item["assertions"]) >= 3
        for assertion in item["assertions"]:
            assert isinstance(assertion, str) and len(assertion.strip()) >= 8

    assert min(seen_ids) == 1
    assert max(seen_ids) == len(seen_ids)


def test_eval_queries_json_schema():
    data = _load_json(QUERIES_FILE)

    assert data.get("skill_name") == "ros2-skill"
    assert isinstance(data.get("eval_queries"), list)
    assert len(data["eval_queries"]) >= 10

    seen_ids = set()
    has_true = False
    has_false = False
    has_note = False

    for item in data["eval_queries"]:
        assert set(item.keys()).issubset(_QUERY_KEYS)

        assert isinstance(item.get("id"), int)
        assert item["id"] not in seen_ids
        seen_ids.add(item["id"])

        assert isinstance(item.get("query"), str) and len(item["query"].strip()) >= 10
        assert isinstance(item.get("should_trigger"), bool)
        if "note" in item:
            assert isinstance(item["note"], str) and item["note"].strip()
            has_note = True

        has_true = has_true or item["should_trigger"]
        has_false = has_false or (not item["should_trigger"])

    # Keep both positive and negative trigger cases in the corpus.
    assert has_true
    assert has_false
    assert has_note
    assert min(seen_ids) == 1
    assert max(seen_ids) == len(seen_ids)
