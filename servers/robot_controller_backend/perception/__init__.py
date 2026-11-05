"""Perception adapters that do not directly command robot hardware."""

from .bench_report_adapter import (
    BenchReportError,
    BenchReportSummary,
    load_bench_report,
    summarize_bench_report,
)

__all__ = [
    "BenchReportError",
    "BenchReportSummary",
    "load_bench_report",
    "summarize_bench_report",
]
