# ocs2_hexbot_github_export

This directory is a GitHub-ready export of the source code from `ocs2_hexbot_ws`.

Included:
- `src/` with Hexbot packages, notes, and the vendored `ocs2` source tree
- top-level helper scripts such as container/runtime build scripts

Excluded:
- `.git/`
- nested repository metadata
- `build*`, `install*`, `log*`
- `recovery_snapshots/`
- `.tmp_upstream/`
- Python cache files

Notes:
- `src/ocs2` has been copied as a real directory so the repository is self-contained.
- The original workspace at `/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws` was not modified.
- This export is suitable for `git init`, review, and upload to GitHub.
