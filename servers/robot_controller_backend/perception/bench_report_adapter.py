"""Read recorded perception reports without connecting them to motor control.

The producer and policy source of truth live in Jetson-RealSense-Perception.
This module validates the stable report envelope and summarizes recorded events
for offline verification. It deliberately has no GPIO, ROS, WebSocket, or motor
imports.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any, Mapping


REPORT_TYPE = "bench_replay_report"
SCHEMA_VERSION = 1
VALID_STATUSES = {"CLEAR", "STOP", "SENSOR_FAULT"}
VALID_OUTCOMES = {"PASS", "FAIL", "SKIP"}
DEFAULT_MAX_BYTES = 25 * 1024 * 1024


class BenchReportError(ValueError):
    """A report cannot be safely consumed by this adapter."""


@dataclass(frozen=True)
class BenchReportSummary:
    recording_name: str
    source: str
    source_label: str
    total_events: int
    status_counts: dict[str, int]
    outcome_counts: dict[str, int]
    latest_status: str
    blocking_events: int
    sensor_fault_events: int

    @property
    def is_hardware_recorded(self) -> bool:
        return self.source == "hardware"

    @property
    def expectations_complete(self) -> bool:
        return self.outcome_counts["SKIP"] == 0


def _object(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, dict):
        raise BenchReportError(f"{label} must be an object")
    return value


def _count_map(value: Any, keys: set[str], label: str) -> dict[str, int]:
    data = _object(value, label)
    if set(data) != keys:
        raise BenchReportError(f"{label} must contain exactly {sorted(keys)}")
    if any(not isinstance(item, int) or isinstance(item, bool) or item < 0 for item in data.values()):
        raise BenchReportError(f"{label} values must be non-negative integers")
    return dict(data)


def _finite_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def summarize_bench_report(report: Any) -> BenchReportSummary:
    root = _object(report, "report")
    if root.get("report_type") != REPORT_TYPE:
        raise BenchReportError(f"report_type must be {REPORT_TYPE!r}")
    if root.get("schema_version") != SCHEMA_VERSION:
        raise BenchReportError(f"unsupported schema_version {root.get('schema_version')!r}")

    recording = _object(root.get("recording"), "recording")
    name = recording.get("name")
    source = recording.get("source")
    source_label = recording.get("source_label")
    if not isinstance(name, str) or not name:
        raise BenchReportError("recording.name must be a non-empty string")
    if source not in {"synthetic", "hardware", "unknown"}:
        raise BenchReportError("recording.source must be synthetic, hardware, or unknown")
    if not isinstance(source_label, str) or not source_label:
        raise BenchReportError("recording.source_label must be a non-empty string")

    events = root.get("events")
    if not isinstance(events, list) or not events:
        raise BenchReportError("events must be a non-empty array")

    observed_statuses = {status: 0 for status in VALID_STATUSES}
    observed_outcomes = {outcome: 0 for outcome in VALID_OUTCOMES}
    previous_timestamp = -math.inf
    for index, raw_event in enumerate(events):
        event = _object(raw_event, f"events[{index}]")
        status = event.get("status")
        outcome = event.get("outcome")
        timestamp = event.get("timestamp_s")
        if status not in VALID_STATUSES:
            raise BenchReportError(f"events[{index}].status is invalid")
        if outcome not in VALID_OUTCOMES:
            raise BenchReportError(f"events[{index}].outcome is invalid")
        if not _finite_number(timestamp) or timestamp < previous_timestamp:
            raise BenchReportError("event timestamps must be finite and non-decreasing")
        previous_timestamp = float(timestamp)
        observed_statuses[status] += 1
        observed_outcomes[outcome] += 1

    summary = _object(root.get("summary"), "summary")
    declared_total = summary.get("total")
    declared_statuses = _count_map(summary.get("statuses"), VALID_STATUSES, "summary.statuses")
    declared_outcomes = _count_map(summary.get("outcomes"), VALID_OUTCOMES, "summary.outcomes")
    if declared_total != len(events):
        raise BenchReportError("summary.total does not match the event count")
    if declared_statuses != observed_statuses:
        raise BenchReportError("summary.statuses does not match the events")
    if declared_outcomes != observed_outcomes:
        raise BenchReportError("summary.outcomes does not match the events")

    return BenchReportSummary(
        recording_name=name,
        source=source,
        source_label=source_label,
        total_events=len(events),
        status_counts=observed_statuses,
        outcome_counts=observed_outcomes,
        latest_status=events[-1]["status"],
        blocking_events=observed_statuses["STOP"] + observed_statuses["SENSOR_FAULT"],
        sensor_fault_events=observed_statuses["SENSOR_FAULT"],
    )


def load_bench_report(path: str | Path, max_bytes: int = DEFAULT_MAX_BYTES) -> BenchReportSummary:
    report_path = Path(path)
    try:
        size = report_path.stat().st_size
    except OSError as exc:
        raise BenchReportError(f"could not inspect {report_path}: {exc}") from exc
    if size > max_bytes:
        raise BenchReportError(f"report exceeds the {max_bytes}-byte local limit")
    try:
        report = json.loads(report_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise BenchReportError(f"could not read report: {exc}") from exc
    return summarize_bench_report(report)
