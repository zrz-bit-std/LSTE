"""Verified per-trial X11 video capture for office benchmark evidence."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time


ROOT = Path(__file__).resolve().parents[3]
RECORDER = ROOT / "scripts/tests/rl_fixed_goal/record_desktop.sh"


@dataclass
class VideoCapture:
    """One recorder process and its evidence paths."""

    process: subprocess.Popen
    stream: object
    output: Path
    log_path: Path


def recorder_duration_seconds(value) -> int:
    """Normalize a trial duration to the recorder's integer contract."""
    return max(1, int(math.ceil(float(value))))


def _log(stream, event: str, **fields) -> None:
    rendered = " ".join("%s=%s" % (key, fields[key]) for key in sorted(fields))
    stream.write(
        "%s [INFO] process=office_video event=%s %s\n"
        % (time.strftime("%Y-%m-%dT%H:%M:%S%z"), event, rendered)
    )
    stream.flush()


def start_capture(run_dir: Path, trial, environment: dict) -> VideoCapture:
    """Start one isolated recorder after the verified node startup barrier."""
    if not RECORDER.is_file():
        raise RuntimeError("shared screen recorder is missing: %s" % RECORDER)
    timestamp = run_dir.name
    video_dir = run_dir / "videos"
    video_dir.mkdir(parents=True, exist_ok=True)
    output = video_dir / (
        "%s_%s_%s_trial%s.mp4"
        % (timestamp, trial.method, trial.level, trial.trial_id)
    )
    log_path = run_dir / (timestamp + "_video_capture.log")
    stream = log_path.open("w", encoding="utf-8", buffering=1)
    # Diagnostic runs may replace the matrix timeout with a float. The shared
    # recorder intentionally accepts positive integer seconds, so round up to
    # cover the complete navigation window without truncating its evidence.
    duration_seconds = recorder_duration_seconds(trial.timeout_seconds)
    command = [
        str(RECORDER),
        "--output", str(output),
        "--duration", str(duration_seconds),
        "--fps", str(trial.video_fps),
        "--label", "method=%s_level=%s_seed=%s" % (
            trial.method, trial.level, trial.seed,
        ),
        "--window-name", trial.video_window,
    ]
    _log(
        stream,
        "capture_scheduled",
        output=output,
        window=trial.video_window,
        fps=trial.video_fps,
        maximum_duration_seconds=duration_seconds,
        requested_duration_seconds=trial.timeout_seconds,
    )
    process = subprocess.Popen(
        command,
        cwd=ROOT,
        env=environment,
        stdin=subprocess.DEVNULL,
        stdout=stream,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    return VideoCapture(process=process, stream=stream, output=output, log_path=log_path)


def finish_capture(capture: VideoCapture | None) -> dict:
    """Stop one recording cleanly, then validate its actual MP4 container."""
    if capture is None:
        return {"video_status": "missing"}
    process = capture.process
    if process.poll() is None:
        # The recorder shell moves a verified partial MP4 into place after its
        # encoder exits. Interrupt the encoder child rather than its whole
        # process group: a group-wide SIGINT can interrupt that shell while it
        # is writing the MP4 trailer.
        children = subprocess.run(
            ["pgrep", "-P", str(process.pid)],
            text=True,
            capture_output=True,
            check=False,
        ).stdout.split()
        for child in children:
            try:
                os.kill(int(child), signal.SIGINT)
            except ProcessLookupError:
                continue
        if not children:
            process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)
    capture.stream.close()
    result = {
        "video_path": str(capture.output),
        "video_capture_log": str(capture.log_path),
        "video_recorder_exit_code": process.returncode,
    }
    try:
        probe = subprocess.run(
            [
                "ffprobe", "-v", "error", "-show_entries",
                "format=duration,size", "-of", "json", str(capture.output),
            ],
            text=True,
            capture_output=True,
            check=True,
        )
        details = json.loads(probe.stdout).get("format") or {}
        duration = float(details.get("duration") or 0.0)
        size = int(details.get("size") or 0)
    except (OSError, ValueError, subprocess.CalledProcessError, json.JSONDecodeError):
        result["video_status"] = "failed"
        return result
    log_text = capture.log_path.read_text(encoding="utf-8", errors="replace")
    completed = (
        "event=record_complete" in log_text
        or "event=record_interrupted" in log_text
    )
    if duration >= 1.0 and size > 1024 and completed:
        result.update({
            "video_status": "recorded",
            "video_duration_seconds": round(duration, 3),
            "video_bytes": size,
        })
    else:
        result["video_status"] = "failed"
    return result
