#!/usr/bin/env python3
"""``notes``: free-form robot memory that outlives the session (dock pose, map update dates, quirks).

Stored in ``.artifacts/notes.json`` next to the named places; the file is not part of any profile, so it
survives profile switches and rescans. ``recall`` matches every query word, case-insensitively.
"""

import json
import os
import time
from pathlib import Path

STORE = Path(__file__).resolve().parent.parent / ".artifacts" / "notes.json"


def _read():
    try:
        data = json.loads(STORE.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {"next_id": 1, "notes": []}
    notes = [n for n in data.get("notes", []) if isinstance(n, dict) and "id" in n and "text" in n] if isinstance(data, dict) else []
    floor = max((n["id"] for n in notes), default=0) + 1
    return {"next_id": max(floor, data.get("next_id", 1)) if isinstance(data, dict) else floor, "notes": notes}


def _load():
    return _read()["notes"]


def _save(state):
    STORE.parent.mkdir(parents=True, exist_ok=True)
    tmp = STORE.with_suffix(".tmp")
    tmp.write_text(json.dumps(state, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    os.replace(tmp, STORE)


def remember(text, tags=None, now=None):
    """Append a note; ids only grow, so a forgotten id is never reused. Returns the stored note."""
    state = _read()
    note = {"id": state["next_id"], "text": text.strip(),
            "tags": sorted({t.strip().lower() for t in (tags or []) if t.strip()}),
            "created_at": now or time.strftime("%Y-%m-%dT%H:%M:%S")}
    _save({"next_id": note["id"] + 1, "notes": state["notes"] + [note]})
    return note


def recall(query="", tag=None, limit=20):
    """Newest first; every word of *query* must appear in the text or a tag."""
    words = [w.lower() for w in (query or "").split()]
    out = []
    for note in reversed(_load()):
        haystack = (note["text"] + " " + " ".join(note.get("tags", []))).lower()
        if tag and tag.lower() not in note.get("tags", []):
            continue
        if all(w in haystack for w in words):
            out.append(note)
    return out[:limit]


def forget(note_id):
    state = _read()
    kept = [n for n in state["notes"] if n["id"] != note_id]
    if len(kept) == len(state["notes"]):
        return False
    _save({"next_id": state["next_id"], "notes": kept})
    return True


def cmd_notes(args):
    from ros2_utils import output
    sub = args.subcommand
    if sub == "remember":
        text = " ".join(args.text).strip()
        if not text:
            return output({"error": "nothing to remember: give the note text"})
        return output({"saved": True, "note": remember(text, args.tag)})
    if sub == "recall":
        found = recall(" ".join(args.query), args.tag, args.limit)
        return output({"notes": found, "count": len(found), "store": str(STORE)})
    if not forget(args.id):
        return output({"error": f"no note with id {args.id}"})
    output({"forgotten": True, "id": args.id})
