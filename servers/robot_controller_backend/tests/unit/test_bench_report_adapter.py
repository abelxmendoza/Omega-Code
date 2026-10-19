"""Offline contract tests for the Jetson perception report consumer."""

from __future__ import annotations

import json
import pathlib
import sys

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from perception.bench_report_adapter import BenchReportError, load_bench_report, summarize_bench_report


def report(*, source="synthetic", events=None):
    events = events or [
        {"timestamp_s": 0.0, "status": "CLEAR", "outcome": "PASS"},
        {"timestamp_s": 1.0, "status": "STOP", "outcome": "SKIP"},
        {"timestamp_s": 2.0, "status": "SENSOR_FAULT", "outcome": "PASS"},
    ]
    statuses = {"CLEAR": 0, "STOP": 0, "SENSOR_FAULT": 0}
    outcomes = {"PASS": 0, "FAIL": 0, "SKIP": 0}
    for event in events:
        statuses[event["status"]] += 1
        outcomes[event["outcome"]] += 1
    return {
        "report_type": "bench_replay_report",
        "schema_version": 1,
        "recording": {
            "name": "fixture.jsonl",
            "source": source,
            "source_label": "Hardware-recorded data" if source == "hardware" else "Synthetic fixture",
            "metadata": {},
        },
        "policy": {},
        "summary": {"total": len(events), "statuses": statuses, "outcomes": outcomes},
        "events": events,
    }


def test_preserves_faults_stops_and_skips_without_motor_side_effects():
    summary = summarize_bench_report(report())
    assert summary.blocking_events == 2
    assert summary.sensor_fault_events == 1
    assert summary.outcome_counts == {"PASS": 2, "FAIL": 0, "SKIP": 1}
    assert not summary.expectations_complete
    assert not summary.is_hardware_recorded


def test_hardware_label_is_not_inferred_from_filename():
    summary = summarize_bench_report(report(source="hardware"))
    assert summary.is_hardware_recorded
    assert summary.source_label == "Hardware-recorded data"


@pytest.mark.parametrize("field,value", [
    ("report_type", "other"),
    ("schema_version", 2),
])
def test_wrong_contract_is_rejected(field, value):
    payload = report()
    payload[field] = value
    with pytest.raises(BenchReportError):
        summarize_bench_report(payload)


def test_summary_tampering_is_rejected():
    payload = report()
    payload["summary"]["outcomes"]["PASS"] = 999
    with pytest.raises(BenchReportError, match="does not match"):
        summarize_bench_report(payload)


def test_out_of_order_timestamps_are_rejected():
    payload = report(events=[
        {"timestamp_s": 1.0, "status": "CLEAR", "outcome": "PASS"},
        {"timestamp_s": 0.5, "status": "CLEAR", "outcome": "PASS"},
    ])
    with pytest.raises(BenchReportError, match="non-decreasing"):
        summarize_bench_report(payload)


def test_loads_local_json_report(tmp_path):
    path = tmp_path / "report.json"
    path.write_text(json.dumps(report()), encoding="utf-8")
    assert load_bench_report(path).latest_status == "SENSOR_FAULT"


def test_size_limit_is_enforced(tmp_path):
    path = tmp_path / "report.json"
    path.write_text(json.dumps(report()), encoding="utf-8")
    with pytest.raises(BenchReportError, match="local limit"):
        load_bench_report(path, max_bytes=10)
