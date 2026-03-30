#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import math
import os
import re
import shutil
import subprocess
import sys
import time
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Any


THIS_FILE = Path(__file__).resolve()
WORKSPACE_ROOT = THIS_FILE.parents[3]
DEFAULT_PROBE_SCRIPT = (
    WORKSPACE_ROOT / "src" / "hexbot_locomotion_mpc" / "scripts" / "run_idle_reference_safety_probe.sh"
)
DEFAULT_TASK_FILE = (
    WORKSPACE_ROOT / "src" / "ocs2_hexbot_legged_robot" / "config" / "mpc" / "task.info"
)
DEFAULT_LOG_DIR = WORKSPACE_ROOT / "log_humble" / "idle_reference_probe"
DEFAULT_RESULTS_ROOT = WORKSPACE_ROOT / "src" / "hexbot_locomotion_mpc" / "results"


def slugify(value: str) -> str:
    lowered = value.strip().lower()
    collapsed = re.sub(r"[^a-z0-9]+", "_", lowered)
    return collapsed.strip("_") or "run"


def replace_bool_setting(text: str, key: str, enabled: bool) -> str:
    pattern = re.compile(rf"(^\s*{re.escape(key)}\s+)(true|false)\b", re.MULTILINE)
    replacement = r"\1" + ("true" if enabled else "false")
    new_text, count = pattern.subn(replacement, text, count=1)
    if count != 1:
        raise RuntimeError(f"Unable to set {key} in task file.")
    return new_text


def replace_scalar_setting(text: str, key: str, value: str) -> str:
    pattern = re.compile(rf"(^\s*{re.escape(key)}\s+)([^\s/]+)", re.MULTILINE)
    replacement = r"\1" + value
    new_text, count = pattern.subn(replacement, text, count=1)
    if count != 1:
        raise RuntimeError(f"Unable to set {key} in task file.")
    return new_text


def format_cli_number(value: float | int) -> str:
    numeric = float(value)
    if math.isfinite(numeric) and abs(numeric - round(numeric)) <= 1.0e-9:
        return str(int(round(numeric)))
    return f"{numeric:g}"


@dataclass(frozen=True)
class ExperimentGroup:
    name: str
    solver: str
    enable_tarsus_branch_constraint: bool
    task_filename: str


GROUPS: tuple[ExperimentGroup, ...] = (
    ExperimentGroup(
        name="Proposed",
        solver="ipm",
        enable_tarsus_branch_constraint=True,
        task_filename="task_proposed_ipm_branch_on.info",
    ),
    ExperimentGroup(
        name="Baseline_SQP",
        solver="sqp",
        enable_tarsus_branch_constraint=False,
        task_filename="task_baseline_sqp_branch_off.info",
    ),
)


@dataclass
class RunOutcome:
    group: str
    group_slug: str
    logical_run_index: int
    attempt_index: int
    label_prefix: str
    run_tag: str | None
    command: list[str]
    started_at: str
    finished_at: str
    duration_sec: float
    exit_code: int | None
    timed_out: bool
    summary_found: bool
    archived_files: list[str]
    stdout_log: str
    stderr_log: str
    summary_file: str | None
    error: str | None


class BatchRunner:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.log_dir = args.log_dir.resolve()
        self.results_root = args.results_root.resolve()
        self.batch_id = args.batch_id or datetime.now().strftime("%Y%m%d_%H%M%S")
        self.batch_root = self.results_root / f"batch_{self.batch_id}"
        self.generated_task_dir = self.batch_root / "_generated_tasks"
        self.manifest_path = self.batch_root / "batch_manifest.json"
        self.index_jsonl_path = self.batch_root / "runs.jsonl"
        self.groups_by_slug = {slugify(group.name): group for group in GROUPS}

    def ensure_layout(self) -> None:
        self.batch_root.mkdir(parents=True, exist_ok=True)
        self.generated_task_dir.mkdir(parents=True, exist_ok=True)
        for group in GROUPS:
            (self.batch_root / slugify(group.name)).mkdir(parents=True, exist_ok=True)

    def write_group_task_file(self, group: ExperimentGroup) -> Path:
        text = self.args.base_task_file.read_text(encoding="utf-8")
        text = replace_bool_setting(
            text,
            "enableTarsusBranchConstraint",
            group.enable_tarsus_branch_constraint,
        )
        text = replace_bool_setting(text, "enabled", False)
        path = self.generated_task_dir / group.task_filename
        path.write_text(text, encoding="utf-8")
        return path

    def capture_existing_run_tags(self, prefix: str) -> set[str]:
        return {
            path.name[: -len("_summary.json")]
            for path in self.log_dir.glob(f"{prefix}_*_summary.json")
        }

    def discover_run_tag(self, prefix: str, started_after: float, before: set[str]) -> str | None:
        candidates: list[tuple[float, str]] = []
        for path in self.log_dir.glob(f"{prefix}_*_summary.json"):
            run_tag = path.name[: -len("_summary.json")]
            if run_tag in before:
                continue
            try:
                mtime = path.stat().st_mtime
            except FileNotFoundError:
                continue
            if mtime + 1.0 < started_after:
                continue
            candidates.append((mtime, run_tag))
        if not candidates:
            return None
        candidates.sort()
        return candidates[-1][1]

    def archive_run_artifacts(self, run_tag: str, group_dir: Path) -> list[str]:
        archived_files: list[str] = []
        for path in sorted(self.log_dir.glob(f"{run_tag}_*")):
            destination = group_dir / path.name
            if destination.exists():
                destination = group_dir / f"{run_tag}_{int(time.time())}_{path.name}"
            try:
                shutil.move(str(path), str(destination))
            except (FileNotFoundError, shutil.Error):
                if path.exists():
                    shutil.copy2(path, destination)
            archived_files.append(str(destination))
        return archived_files

    def write_text_log(self, path: Path, content: str) -> str:
        path.write_text(content, encoding="utf-8")
        return str(path)

    def append_outcome(self, outcome: RunOutcome) -> None:
        with self.index_jsonl_path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(asdict(outcome), ensure_ascii=False) + "\n")

    def run(self) -> int:
        self.ensure_layout()
        requested_groups = self.select_groups()
        task_files = {
            slugify(group.name): self.write_group_task_file(group)
            for group in requested_groups
        }

        batch_manifest = {
            "batch_id": self.batch_id,
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "base_task_file": str(self.args.base_task_file.resolve()),
            "probe_script": str(self.args.probe_script.resolve()),
            "log_dir": str(self.log_dir),
            "results_root": str(self.batch_root),
            "runs_per_group": int(self.args.runs),
            "duration_sec": float(self.args.duration),
            "probe_duration_sec": float(self.args.probe_duration),
            "linear_x": float(self.args.linear_x),
            "angular_z": float(self.args.angular_z),
            "prime_duration_sec": float(self.args.prime_duration),
            "prime_max_final_error_rad": float(self.args.prime_max_final_error_rad),
            "translation_lookahead_sec": (
                float(self.args.translation_lookahead)
                if self.args.translation_lookahead is not None
                else None
            ),
            "rotation_lookahead_sec": (
                float(self.args.rotation_lookahead)
                if self.args.rotation_lookahead is not None
                else None
            ),
            "moving_goal_reference_file": (
                str(self.args.moving_goal_reference_file.resolve())
                if self.args.moving_goal_reference_file is not None
                else None
            ),
            "auto_load_moving_goal_reference": (
                bool(self.args.auto_load_moving_goal_reference)
                if self.args.auto_load_moving_goal_reference is not None
                else None
            ),
            "use_observed_joint_target_while_moving": (
                bool(self.args.use_observed_joint_target_while_moving)
                if self.args.use_observed_joint_target_while_moving is not None
                else None
            ),
            "max_attempts": int(self.args.max_attempts),
            "groups": [
                {
                    **asdict(group),
                    "group_slug": slugify(group.name),
                    "task_file": str(task_files[slugify(group.name)]),
                }
                for group in requested_groups
            ],
        }
        self.manifest_path.write_text(
            json.dumps(batch_manifest, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )

        overall_failures = 0
        for group in requested_groups:
            group_slug = slugify(group.name)
            group_dir = self.batch_root / group_slug
            task_file = task_files[group_slug]
            for logical_run_index in range(1, int(self.args.runs) + 1):
                label_prefix = f"{self.batch_id}_{group_slug}_r{logical_run_index:02d}"
                success = False
                for attempt_index in range(1, int(self.args.max_attempts) + 1):
                    outcome = self.execute_single_attempt(
                        group=group,
                        task_file=task_file,
                        group_dir=group_dir,
                        logical_run_index=logical_run_index,
                        attempt_index=attempt_index,
                        label_prefix=label_prefix,
                    )
                    self.append_outcome(outcome)
                    if outcome.summary_found and outcome.exit_code == 0 and not outcome.timed_out:
                        success = True
                        break
                if not success:
                    overall_failures += 1

        return 0 if overall_failures == 0 else 1

    def select_groups(self) -> list[ExperimentGroup]:
        if not self.args.groups:
            return list(GROUPS)
        selected: list[ExperimentGroup] = []
        for raw_name in self.args.groups:
            group = self.groups_by_slug.get(slugify(raw_name))
            if group is None:
                available = ", ".join(group.name for group in GROUPS)
                raise SystemExit(f"Unknown group '{raw_name}'. Available groups: {available}")
            selected.append(group)
        return selected

    def execute_single_attempt(
        self,
        *,
        group: ExperimentGroup,
        task_file: Path,
        group_dir: Path,
        logical_run_index: int,
        attempt_index: int,
        label_prefix: str,
    ) -> RunOutcome:
        start_epoch = time.time()
        started_at = datetime.now().isoformat(timespec="seconds")
        stdout_log = group_dir / f"{label_prefix}_attempt{attempt_index:02d}_stdout.log"
        stderr_log = group_dir / f"{label_prefix}_attempt{attempt_index:02d}_stderr.log"
        before = self.capture_existing_run_tags(label_prefix)
        command = [
            "bash",
            str(self.args.probe_script.resolve()),
            "--solver",
            group.solver,
            "--task-file",
            str(task_file),
            "--label",
            label_prefix,
            "--duration",
            format_cli_number(self.args.duration),
            "--probe-duration",
            format_cli_number(self.args.probe_duration),
            "--linear-x",
            format_cli_number(self.args.linear_x),
            "--angular-z",
            format_cli_number(self.args.angular_z),
            "--prime-duration",
            format_cli_number(self.args.prime_duration),
            "--prime-max-final-error-rad",
            format_cli_number(self.args.prime_max_final_error_rad),
        ]
        if self.args.translation_lookahead is not None:
            command.extend(
                [
                    "--translation-lookahead",
                    format_cli_number(self.args.translation_lookahead),
                ]
            )
        if self.args.rotation_lookahead is not None:
            command.extend(
                [
                    "--rotation-lookahead",
                    format_cli_number(self.args.rotation_lookahead),
                ]
            )
        if self.args.moving_goal_reference_file is not None:
            command.extend(
                [
                    "--moving-goal-reference-file",
                    str(self.args.moving_goal_reference_file.resolve()),
                ]
            )
        if self.args.auto_load_moving_goal_reference is not None:
            command.extend(
                [
                    "--auto-load-moving-goal-reference",
                    "true" if self.args.auto_load_moving_goal_reference else "false",
                ]
            )
        if self.args.use_observed_joint_target_while_moving is not None:
            command.extend(
                [
                    "--use-observed-joint-target-while-moving",
                    "true"
                    if self.args.use_observed_joint_target_while_moving
                    else "false",
                ]
            )

        timed_out = False
        exit_code: int | None = None
        error: str | None = None
        stdout_text = ""
        stderr_text = ""
        run_tag: str | None = None
        archived_files: list[str] = []
        summary_file: str | None = None

        env = os.environ.copy()
        env.setdefault("PYTHONUNBUFFERED", "1")

        try:
            completed = subprocess.run(
                command,
                cwd=str(WORKSPACE_ROOT),
                capture_output=True,
                text=True,
                timeout=float(self.args.timeout_sec),
                env=env,
                check=False,
            )
            exit_code = completed.returncode
            stdout_text = completed.stdout
            stderr_text = completed.stderr
        except subprocess.TimeoutExpired as exc:
            timed_out = True
            exit_code = None
            stdout_text = exc.stdout if isinstance(exc.stdout, str) else (exc.stdout or b"").decode("utf-8", errors="replace")
            stderr_text = exc.stderr if isinstance(exc.stderr, str) else (exc.stderr or b"").decode("utf-8", errors="replace")
            error = f"Timed out after {self.args.timeout_sec:.1f}s"
        except Exception as exc:  # noqa: BLE001
            exit_code = None
            error = str(exc)

        self.write_text_log(stdout_log, stdout_text)
        self.write_text_log(stderr_log, stderr_text)

        run_tag = self.discover_run_tag(label_prefix, start_epoch, before)
        if run_tag is not None:
            archived_files = self.archive_run_artifacts(run_tag, group_dir)
            summary_path = group_dir / f"{run_tag}_summary.json"
            if summary_path.exists():
                summary_file = str(summary_path)

        summary_found = summary_file is not None
        if not summary_found and error is None and exit_code not in (0, None):
            error = f"Probe script exited with code {exit_code}"
        if not summary_found and error is None:
            error = "Missing summary artifact after run."

        finished_at = datetime.now().isoformat(timespec="seconds")
        duration_sec = time.time() - start_epoch
        return RunOutcome(
            group=group.name,
            group_slug=slugify(group.name),
            logical_run_index=logical_run_index,
            attempt_index=attempt_index,
            label_prefix=label_prefix,
            run_tag=run_tag,
            command=command,
            started_at=started_at,
            finished_at=finished_at,
            duration_sec=duration_sec,
            exit_code=exit_code,
            timed_out=timed_out,
            summary_found=summary_found,
            archived_files=archived_files,
            stdout_log=str(stdout_log),
            stderr_log=str(stderr_log),
            summary_file=summary_file,
            error=error,
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Batch-run Hexbot closed-loop walking experiments and archive each run "
            "into per-baseline result folders."
        )
    )
    parser.add_argument("--runs", type=int, default=10)
    parser.add_argument("--max-attempts", type=int, default=2)
    parser.add_argument("--duration", type=float, default=12.0)
    parser.add_argument("--probe-duration", type=float, default=8.0)
    parser.add_argument("--linear-x", type=float, default=0.03)
    parser.add_argument("--angular-z", type=float, default=0.0)
    parser.add_argument("--prime-duration", type=float, default=10.0)
    parser.add_argument("--prime-max-final-error-rad", type=float, default=0.45)
    parser.add_argument("--translation-lookahead", type=float, default=None)
    parser.add_argument("--rotation-lookahead", type=float, default=None)
    parser.add_argument("--moving-goal-reference-file", type=Path, default=None)
    parser.add_argument("--auto-load-moving-goal-reference", default=None)
    parser.add_argument("--use-observed-joint-target-while-moving", default=None)
    parser.add_argument("--timeout-sec", type=float, default=240.0)
    parser.add_argument("--batch-id", default="")
    parser.add_argument("--groups", nargs="*", default=[])
    parser.add_argument("--probe-script", type=Path, default=DEFAULT_PROBE_SCRIPT)
    parser.add_argument("--base-task-file", type=Path, default=DEFAULT_TASK_FILE)
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--results-root", type=Path, default=DEFAULT_RESULTS_ROOT)
    args = parser.parse_args()

    if args.runs < 1:
        raise SystemExit("--runs must be >= 1")
    if args.max_attempts < 1:
        raise SystemExit("--max-attempts must be >= 1")
    if args.duration <= 0.0 or args.probe_duration <= 0.0:
        raise SystemExit("--duration and --probe-duration must be > 0")
    if args.prime_duration <= 0.0:
        raise SystemExit("--prime-duration must be > 0")
    if not args.probe_script.is_file():
        raise SystemExit(f"Probe script not found: {args.probe_script}")
    if not args.base_task_file.is_file():
        raise SystemExit(f"Base task file not found: {args.base_task_file}")
    if args.moving_goal_reference_file is not None and not args.moving_goal_reference_file.is_file():
        raise SystemExit(
            f"Moving-goal reference file not found: {args.moving_goal_reference_file}"
        )
    if args.auto_load_moving_goal_reference is not None:
        args.auto_load_moving_goal_reference = str(
            args.auto_load_moving_goal_reference
        ).strip().lower() in {"1", "true", "yes", "on"}
    if args.use_observed_joint_target_while_moving is not None:
        args.use_observed_joint_target_while_moving = str(
            args.use_observed_joint_target_while_moving
        ).strip().lower() in {"1", "true", "yes", "on"}
    args.batch_id = slugify(args.batch_id) if args.batch_id else ""
    return args


def main() -> None:
    args = parse_args()
    runner = BatchRunner(args)
    raise SystemExit(runner.run())


if __name__ == "__main__":
    main()
