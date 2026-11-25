#!/usr/bin/env python3
"""Named places on disk, readable without importing rclpy (``plan`` compiles goals with them)."""

import json
import pathlib

PLACE_DIR = pathlib.Path(__file__).resolve().parent.parent / ".artifacts" / "places"


def list_places(place_dir=None):
    place_dir = place_dir or PLACE_DIR
    if not place_dir.exists():
        return []
    places = []
    for path in sorted(place_dir.glob("*.json")):
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except Exception:
            continue
        if isinstance(data, dict) and data.get("name"):
            places.append(data)
    return places
