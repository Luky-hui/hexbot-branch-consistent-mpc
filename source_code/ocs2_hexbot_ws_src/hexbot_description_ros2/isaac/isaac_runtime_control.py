#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import sys

from isaac_vscode_exec import _execute_code


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Control a running Isaac Sim instance through the "
            "isaacsim.code_editor.vscode socket."
        )
    )
    parser.add_argument(
        "action",
        choices=[
            "status",
            "stop",
            "play",
            "reload-stage",
            "reload-and-play",
            "articulation-roots",
        ],
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8226)
    parser.add_argument("--timeout", type=float, default=5.0)
    return parser.parse_args()


def _code_for_action(action: str) -> str:
    if action == "status":
        return """
import omni.timeline
import omni.usd
t = omni.timeline.get_timeline_interface()
ctx = omni.usd.get_context()
print({
    'stage_url': ctx.get_stage_url(),
    'playing': bool(t.is_playing()),
    'stopped': bool(t.is_stopped()),
})
"""
    if action == "stop":
        return """
import omni.timeline
t = omni.timeline.get_timeline_interface()
t.stop()
print({
    'action': 'stop',
    'playing': bool(t.is_playing()),
    'stopped': bool(t.is_stopped()),
})
"""
    if action == "play":
        return """
import omni.timeline
t = omni.timeline.get_timeline_interface()
t.play()
print({
    'action': 'play',
    'playing': bool(t.is_playing()),
    'stopped': bool(t.is_stopped()),
})
"""
    if action == "reload-stage":
        return """
import omni.usd
ctx = omni.usd.get_context()
url = ctx.get_stage_url()
ctx.open_stage(url)
print({
    'action': 'reload-stage',
    'stage_url': ctx.get_stage_url(),
})
"""
    if action == "reload-and-play":
        return """
import omni.timeline
import omni.usd
ctx = omni.usd.get_context()
url = ctx.get_stage_url()
ctx.open_stage(url)
t = omni.timeline.get_timeline_interface()
t.play()
print({
    'action': 'reload-and-play',
    'stage_url': ctx.get_stage_url(),
    'playing': bool(t.is_playing()),
    'stopped': bool(t.is_stopped()),
})
"""
    if action == "articulation-roots":
        return """
from pxr import UsdPhysics
import omni.usd
stage = omni.usd.get_context().get_stage()
paths = []
for prim in stage.Traverse():
    if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        paths.append(str(prim.GetPath()))
print({'action': 'articulation-roots', 'paths': paths})
"""
    raise SystemExit(f"Unsupported action: {action}")


def main() -> None:
    args = _parse_args()
    reply = _execute_code(
        host=args.host,
        port=args.port,
        timeout=args.timeout,
        code=_code_for_action(args.action),
    )
    print(json.dumps(reply, ensure_ascii=False, indent=2))
    if reply.get("status") != "ok":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
