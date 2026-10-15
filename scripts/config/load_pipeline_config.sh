#!/usr/bin/env bash
set -euo pipefail

# Print shell-safe CFG_* assignments for the standard configuration and one
# optional experiment overlay. Callers keep environment variables as the last
# precedence layer; this helper only makes the YAML inheritance explicit.
WS="${LSTE_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
BASE_CONFIG="$WS/scripts/config/pipeline_defaults.yaml"
OVERLAY_CONFIG="${1:-$BASE_CONFIG}"

python3 - "$BASE_CONFIG" "$OVERLAY_CONFIG" <<'PY'
import shlex
import sys
from pathlib import Path

try:
    import yaml
except ImportError:
    raise SystemExit(0)


def read_mapping(path: Path):
    if not path.is_file():
        return {}
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if not isinstance(data, dict):
        raise SystemExit("pipeline configuration must be a YAML mapping: %s" % path)
    return data


base_path = Path(sys.argv[1]).resolve()
overlay_path = Path(sys.argv[2]).resolve()
merged = read_mapping(base_path)
if overlay_path != base_path:
    merged.update(read_mapping(overlay_path))

for key, value in merged.items():
    if isinstance(value, (str, int, float)):
        print("CFG_%s=%s" % (key, shlex.quote(str(value))))
PY
