#!/usr/bin/env python3
"""安全执行 hexbot Isaac 资产修复流程。

默认运行在预览模式：

- `reimport_hexbot_urdf_asset.py` 以 `--dry-run` 运行
- `synthesize_hexbot_foot_colliders.py` 以 `--dry-run` 运行
- `apply_hexbot_isaac_drives.py` 以 `--dry-run` 运行
- `inspect_hexbot_foot_prims.py` 和 `audit_hexbot_isaac_setup.py` 只读执行

只有显式传入 `--write` 时，才允许写回 USD。
建议用 Isaac Sim 自带的 Python 运行此脚本。
"""

from __future__ import annotations

import argparse
import importlib
import sys
from contextlib import contextmanager
from pathlib import Path
from typing import Iterator


THIS_DIR = Path(__file__).resolve().parent
DEFAULT_URDF_PATH = THIS_DIR.parent / "urdf" / "hexbot_isaac_rooted.urdf"
DEFAULT_USD_PATH = (
    THIS_DIR.parent
    / "urdf"
    / "hexbot_isaac_rooted_reimport"
    / "hexbot_isaac_rooted_reimport.usd"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="执行六足机器人 Isaac 资产修复流程。")
    parser.add_argument(
        "--urdf",
        type=Path,
        default=DEFAULT_URDF_PATH,
        help="待导入的 URDF 路径。",
    )
    parser.add_argument(
        "--usd",
        type=Path,
        default=DEFAULT_USD_PATH,
        help="待检查或写回的 USD 路径。",
    )
    parser.add_argument(
        "--write",
        action="store_true",
        help="允许写回 USD。默认不写盘，只做预览和检查。",
    )
    parser.add_argument(
        "--skip-reimport",
        action="store_true",
        help="跳过 URDF 重新导入步骤。",
    )
    parser.add_argument(
        "--skip-inspect",
        action="store_true",
        help="跳过脚端 prim 结构检查步骤。",
    )
    parser.add_argument(
        "--no-strict-audit",
        action="store_true",
        help="审计失败时不使用严格退出码。",
    )
    return parser.parse_args()


@contextmanager
def temporary_argv(argv: list[str]) -> Iterator[None]:
    original_argv = sys.argv[:]
    sys.argv = argv
    try:
        yield
    finally:
        sys.argv = original_argv


def run_module_main(module_name: str, argv: list[str]) -> int:
    module = importlib.import_module(module_name)
    module = importlib.reload(module)
    main_fn = getattr(module, "main", None)
    if not callable(main_fn):
        raise RuntimeError(f"{module_name} 未暴露可调用的 main()")

    with temporary_argv(argv):
        result = main_fn()

    if result is None:
        return 0
    return int(result)


def main() -> int:
    args = parse_args()
    urdf_path = args.urdf.resolve()
    usd_path = args.usd.resolve()
    write_mode = bool(args.write)
    strict_audit = not args.no_strict_audit

    print("=" * 60)
    print("Hexbot Isaac 资产修复流程")
    print(f"URDF: {urdf_path}")
    print(f"USD:  {usd_path}")
    print(f"模式: {'写回模式' if write_mode else '预览模式(不写盘)'}")

    steps: list[tuple[str, str, list[str]]] = []
    if not args.skip_reimport:
        reimport_argv = [
            "reimport_hexbot_urdf_asset.py",
            "--urdf",
            str(urdf_path),
            "--usd-out",
            str(usd_path),
        ]
        if not write_mode:
            reimport_argv.append("--dry-run")
        steps.append(("步骤 1: 重新导入 URDF", "reimport_hexbot_urdf_asset", reimport_argv))

    if not args.skip_inspect:
        steps.append(
            (
                "步骤 2: 检查脚端 prim 结构",
                "inspect_hexbot_foot_prims",
                [
                    "inspect_hexbot_foot_prims.py",
                    "--urdf",
                    str(urdf_path),
                    "--usd",
                    str(usd_path),
                ],
            )
        )

    synth_argv = [
        "synthesize_hexbot_foot_colliders.py",
        "--urdf",
        str(urdf_path),
        "--usd",
        str(usd_path),
    ]
    if not write_mode:
        synth_argv.append("--dry-run")
    steps.append(("步骤 3: 生成脚端碰撞球", "synthesize_hexbot_foot_colliders", synth_argv))

    apply_argv = [
        "apply_hexbot_isaac_drives.py",
        "--urdf",
        str(urdf_path),
        "--usd",
        str(usd_path),
    ]
    if not write_mode:
        apply_argv.append("--dry-run")
    steps.append(("步骤 4: 写入关节驱动与脚端材质", "apply_hexbot_isaac_drives", apply_argv))

    audit_argv = [
        "audit_hexbot_isaac_setup.py",
        "--urdf",
        str(urdf_path),
        "--usd",
        str(usd_path),
    ]
    if strict_audit:
        audit_argv.append("--strict")
    steps.append(("步骤 5: 严格审计当前 USD", "audit_hexbot_isaac_setup", audit_argv))

    for title, module_name, argv in steps:
        print("")
        print("=" * 60)
        print(title)
        print("命令参数: " + " ".join(argv[1:]))
        exit_code = run_module_main(module_name, argv)
        if exit_code != 0:
            raise RuntimeError(f"{module_name} 执行失败，exit_code={exit_code}")

    print("")
    print("=" * 60)
    print("流程结束。")
    if write_mode:
        print("本次流程已允许写回 USD。")
    else:
        print("本次流程未写回 USD，只做了预览与检查。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
