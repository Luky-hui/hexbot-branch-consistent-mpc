#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import socket
import sys
from pathlib import Path


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Execute Python code inside a running Isaac Sim instance through "
            "the isaacsim.code_editor.vscode integration socket."
        )
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8226)
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--code", help="Inline Python code to execute")
    parser.add_argument("--file", type=Path, help="Python file to execute")
    parser.add_argument(
        "--raw",
        action="store_true",
        help="Print the raw JSON reply instead of only the output payload",
    )
    return parser.parse_args()


def _load_code(args: argparse.Namespace) -> str:
    if args.code and args.file:
        raise SystemExit("Only one of --code or --file may be used.")
    if args.file:
        return args.file.expanduser().resolve().read_text(encoding="utf-8")
    if args.code:
        return args.code
    if not sys.stdin.isatty():
        return sys.stdin.read()
    raise SystemExit("Provide --code, --file, or pipe code through stdin.")


def _execute_code(host: str, port: int, timeout: float, code: str) -> dict:
    client = socket.socket()
    client.settimeout(timeout)
    client.connect((host, port))
    client.sendall(code.encode("utf-8"))

    chunks = []
    try:
        while True:
            data = client.recv(65536)
            if not data:
                break
            chunks.append(data)
    except socket.timeout:
        pass
    finally:
        client.close()

    raw = b"".join(chunks).decode("utf-8", errors="replace")
    if not raw:
        raise SystemExit("Isaac Sim returned an empty reply.")
    try:
        return json.loads(raw)
    except json.JSONDecodeError as exc:
        raise SystemExit(f"Failed to parse Isaac Sim reply as JSON: {exc}\n{raw}")


def main() -> None:
    args = _parse_args()
    code = _load_code(args)
    reply = _execute_code(args.host, args.port, args.timeout, code)

    if args.raw:
        print(json.dumps(reply, ensure_ascii=False, indent=2))
    else:
        output = reply.get("output", "")
        if output:
            print(output)

    if reply.get("status") != "ok":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
