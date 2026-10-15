#!/usr/bin/env bash
set -euo pipefail

# Capture the actual X11 desktop so Gazebo and Live RViz remain the source of
# truth for a demonstration. This helper is launched by run_tmux.sh, which
# redirects its stdout/stderr to the formal per-run screen_record log.

usage() {
  cat <<'EOF'
Usage: record_desktop.sh --output FILE --duration SECONDS --fps FPS --label TEXT [--window-name NAME]
EOF
}

OUTPUT=""
DURATION=""
FPS="30"
LABEL=""
WINDOW_NAME=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --output)
      OUTPUT="$2"
      shift 2
      ;;
    --duration)
      DURATION="$2"
      shift 2
      ;;
    --fps)
      FPS="$2"
      shift 2
      ;;
    --label)
      LABEL="$2"
      shift 2
      ;;
    --window-name)
      WINDOW_NAME="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[error] Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ -z "$OUTPUT" || -z "$DURATION" || -z "$LABEL" ]]; then
  echo "[error] --output, --duration, and --label are required." >&2
  usage >&2
  exit 2
fi
if ! command -v ffmpeg >/dev/null; then
  echo "[error] ffmpeg is required for screen recording." >&2
  exit 1
fi
if ! [[ "$DURATION" =~ ^[1-9][0-9]*$ && "$FPS" =~ ^[1-9][0-9]*$ ]]; then
  echo "[error] duration and fps must be positive integers." >&2
  exit 2
fi

DISPLAY_NAME="${DISPLAY:-:0}"
if [[ ! "$DISPLAY_NAME" =~ \.[0-9]+$ ]]; then
  DISPLAY_NAME="${DISPLAY_NAME}.0"
fi

ROOT_INFO="$(xwininfo -root -display "$DISPLAY_NAME")"
WIDTH="$(awk '/Width:/ { print $2; exit }' <<<"$ROOT_INFO")"
HEIGHT="$(awk '/Height:/ { print $2; exit }' <<<"$ROOT_INFO")"
if ! [[ "$WIDTH" =~ ^[1-9][0-9]*$ && "$HEIGHT" =~ ^[1-9][0-9]*$ ]]; then
  echo "[error] Unable to determine X11 screen geometry for $DISPLAY_NAME." >&2
  exit 1
fi

FONT_FILE="$(fc-match -f '%{file}' DejaVuSans | head -n 1)"
if [[ ! -f "$FONT_FILE" ]]; then
  echo "[error] No usable DejaVu Sans font found for the video label." >&2
  exit 1
fi

mkdir -p "$(dirname "$OUTPUT")"
PARTIAL_OUTPUT="${OUTPUT%.mp4}.partial.mp4"
rm -f "$PARTIAL_OUTPUT"

# drawtext uses ':' as an option separator. Keep the generated label ASCII and
# limited to characters that have no special meaning in the filter expression.
SAFE_LABEL="$(sed 's/[^A-Za-z0-9 .,_=()\/-]/_/g' <<<"$LABEL")"
VIDEO_FILTER="drawbox=x=12:y=12:w=iw-24:h=52:color=black@0.65:t=fill,drawtext=fontfile='${FONT_FILE}':text='${SAFE_LABEL}':fontcolor=white:fontsize=22:x=24:y=28"

CAPTURE_TARGET="desktop"
CAPTURE_BACKEND="ffmpeg"
CAPTURE_X=0
CAPTURE_Y=0
if [[ -n "$WINDOW_NAME" && "$WINDOW_NAME" != "desktop" ]]; then
  if [[ "$WINDOW_NAME" == "RViz" ]]; then
    # Do not exit awk early: with pipefail that would leave xwininfo writing
    # into a closed pipe and make this otherwise valid lookup fail.
    WINDOW_ID="$(xwininfo -root -tree -display "$DISPLAY_NAME" | awk '/ - RViz"/ && !found { print $1; found=1 }')"
  else
    WINDOW_ID="$(xwininfo -name "$WINDOW_NAME" -display "$DISPLAY_NAME" | awk '/Window id:/ { print $4; exit }')"
  fi
  WINDOW_INFO="$(xwininfo -id "$WINDOW_ID" -display "$DISPLAY_NAME")"
  WIDTH="$(awk '/Width:/ { print $2; exit }' <<<"$WINDOW_INFO")"
  HEIGHT="$(awk '/Height:/ { print $2; exit }' <<<"$WINDOW_INFO")"
  CAPTURE_X="$(awk '/Absolute upper-left X:/ { print $4; exit }' <<<"$WINDOW_INFO")"
  CAPTURE_Y="$(awk '/Absolute upper-left Y:/ { print $4; exit }' <<<"$WINDOW_INFO")"
  if ! [[ "$WINDOW_ID" =~ ^0x[0-9a-fA-F]+$ && "$WIDTH" =~ ^[1-9][0-9]*$ && "$HEIGHT" =~ ^[1-9][0-9]*$ && "$CAPTURE_X" =~ ^-?[0-9]+$ && "$CAPTURE_Y" =~ ^-?[0-9]+$ ]]; then
    echo "[error] Unable to resolve visible X11 window: $WINDOW_NAME" >&2
    exit 1
  fi
  CAPTURE_TARGET="window:${WINDOW_NAME}:${WINDOW_ID}"
fi

printf '%s [INFO] [screen_record] event=record_start output=%s backend=%s display=%s target=%s geometry=%sx%s fps=%s duration_seconds=%s label=%s\n' \
  "$(date '+%Y-%m-%d %H:%M:%S.%3N')" "$OUTPUT" "$CAPTURE_BACKEND" "$DISPLAY_NAME" "$CAPTURE_TARGET" "$WIDTH" "$HEIGHT" "$FPS" "$DURATION" "$SAFE_LABEL"

RECORD_COMMAND=(
  ffmpeg -hide_banner -nostdin -y
  -f x11grab -draw_mouse 0 -framerate "$FPS" -video_size "${WIDTH}x${HEIGHT}"
  -i "${DISPLAY_NAME}+${CAPTURE_X},${CAPTURE_Y}" -t "$DURATION"
  -vf "$VIDEO_FILTER" -c:v libx264 -preset veryfast -crf 20 -pix_fmt yuv420p
  -movflags +faststart "$PARTIAL_OUTPUT"
)

RECORD_PID=""
finish_interrupted_recording() {
  if [[ -n "$RECORD_PID" ]] && kill -0 "$RECORD_PID" 2>/dev/null; then
    kill -INT "$RECORD_PID" 2>/dev/null || true
    wait "$RECORD_PID" 2>/dev/null || true
  fi
  if [[ -s "$PARTIAL_OUTPUT" ]]; then
    mv -f "$PARTIAL_OUTPUT" "$OUTPUT"
    printf '%s [INFO] [screen_record] event=record_interrupted output=%s bytes=%s\n' \
      "$(date '+%Y-%m-%d %H:%M:%S.%3N')" "$OUTPUT" "$(stat -c '%s' "$OUTPUT")"
  else
    printf '%s [ERROR] [screen_record] event=record_interrupted_without_output output=%s\n' \
      "$(date '+%Y-%m-%d %H:%M:%S.%3N')" "$OUTPUT" >&2
  fi
  exit 0
}
trap finish_interrupted_recording INT TERM

"${RECORD_COMMAND[@]}" &
RECORD_PID="$!"
if wait "$RECORD_PID"; then
  mv -f "$PARTIAL_OUTPUT" "$OUTPUT"
  printf '%s [INFO] [screen_record] event=record_complete output=%s bytes=%s\n' \
    "$(date '+%Y-%m-%d %H:%M:%S.%3N')" "$OUTPUT" "$(stat -c '%s' "$OUTPUT")"
else
  # A recorder launched in its own process group can receive SIGINT directly
  # while its shell is waiting. ffmpeg then writes a valid MP4 trailer but
  # exits non-zero before this script can enter the signal trap. Preserve that
  # verified partial recording; malformed partial files remain failures.
  if [[ -s "$PARTIAL_OUTPUT" ]] && command -v ffprobe >/dev/null 2>&1 \
      && ffprobe -v error -show_entries format=duration -of default=noprint_wrappers=1 "$PARTIAL_OUTPUT" >/dev/null 2>&1; then
    mv -f "$PARTIAL_OUTPUT" "$OUTPUT"
    printf '%s [INFO] [screen_record] event=record_interrupted output=%s bytes=%s\n' \
      "$(date '+%Y-%m-%d %H:%M:%S.%3N')" "$OUTPUT" "$(stat -c '%s' "$OUTPUT")"
    exit 0
  fi
  rm -f "$PARTIAL_OUTPUT"
  printf '%s [ERROR] [screen_record] event=record_failed output=%s\n' \
    "$(date '+%Y-%m-%d %H:%M:%S.%3N')" "$OUTPUT" >&2
  exit 1
fi
