#!/usr/bin/env bash
# Copy every failure-evidence snapshot produced by a suite into a stable
# diagnostics tree, so the artifacts survive even if the run directory is
# later cleaned.  Idempotent: only missing files are copied.
set -uo pipefail

WS="${LSTE_WS:-/home/zrz/work/lste_ws}"
ROOT="${1:?usage: collect_failure_snapshots.sh <suite_root>}"
OUT="${2:-$WS/runtime/diagnostics/20261031_four_scenario}"

mkdir -p "$OUT"

# Metrics directories referenced by the suite's runner logs.
mapfile -t RUN_DIRS < <(
  grep -h '^run_directory=' "$ROOT"/*_runner.log 2>/dev/null | sed 's/^run_directory=//' | sort -u
)

for RUN_DIR in "${RUN_DIRS[@]}"; do
  [[ -d "$RUN_DIR" ]] || continue
  CASE=$(basename "$(dirname "$(dirname "$RUN_DIR")")")
  DEST="$OUT/$CASE/$(basename "$RUN_DIR")"
  mkdir -p "$DEST"
  for FILE in "$RUN_DIR"/*_failure_[0-9][0-9][0-9][0-9].json; do
    [[ -e "$FILE" ]] || continue
    NAME=$(basename "$FILE")
    [[ -e "$DEST/$NAME" ]] || cp "$FILE" "$DEST/$NAME"
  done
  for EXTRA in "$RUN_DIR"/*_failure_report.md "$RUN_DIR"/*_failure_report.json; do
    [[ -e "$EXTRA" ]] || continue
    NAME=$(basename "$EXTRA")
    [[ -e "$DEST/$NAME" ]] || cp "$EXTRA" "$DEST/$NAME"
  done
  COUNT=$(ls "$DEST"/*_failure_[0-9][0-9][0-9][0-9].json 2>/dev/null | wc -l)
  printf '%s\t%s\t%s\n' "$CASE" "$(basename "$RUN_DIR")" "$COUNT" > "$DEST/failure_count.tsv"
done

echo "collected into $OUT"
find "$OUT" -name failure_count.tsv -exec cat {} \; 2>/dev/null
