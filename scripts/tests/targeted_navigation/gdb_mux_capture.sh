#!/usr/bin/env bash
set -uo pipefail
if (($# == 0)); then
  echo "gdb_mux_capture: missing target command" >&2
  exit 64
fi
log="$(printenv LSTE_CMD_VEL_MUX_GDB_LOG 2>/dev/null || true)"
if [[ -z "$log" ]]; then
  log="$HOME/.ros/lste_cmd_vel_mux_gdb.log"
fi
mkdir -p "$(dirname "$log")"
{
  printf '[%s] gdb mux start pid=%s target=' "$(date --iso-8601=seconds)" "$$"
  printf '%q ' "$@"
  printf '\n'
} >>"$log"
set +e
/usr/bin/gdb --quiet --batch --return-child-result \
  -ex 'set pagination off' \
  -ex 'handle SIGPIPE nostop noprint pass' \
  -ex run \
  -ex 'thread apply all bt full' \
  --args /usr/bin/python3 "$@" >>"$log" 2>&1
status=$?
set -e
printf '[%s] gdb mux exit status=%s\n' "$(date --iso-8601=seconds)" "$status" >>"$log"
if grep -Eq 'Program received signal SIGSEGV|Thread .* received signal SIGSEGV' "$log"; then
  exit 139
fi
if grep -Eq 'Program received signal SIGABRT|Thread .* received signal SIGABRT' "$log"; then
  exit 134
fi
exit "$status"
