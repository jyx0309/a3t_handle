#!/usr/bin/env python3
"""Read-only summary of SDK diagnostics. Never connects to a controller."""
import argparse
import csv
import json
from pathlib import Path


def summarize(records):
    pv = [r for r in records if r.get("type") == "pv_tracking"]
    pv_gaps = sorted(r["send_gap_ms"] for r in pv if r.get("send_gap_ms", -1) > 0)
    pv_errors = [r["max_error_rad"] for r in pv if r.get("valid") and isinstance(r.get("max_error_rad"), (int, float))]
    frames = [r for r in records if r.get("type") == "mit_result"]
    inputs = [r for r in records if r.get("type") == "mit_input"]
    gaps = sorted(r["send_gap_ms"] for r in frames
                  if r.get("frame") != 1 and isinstance(r.get("send_gap_ms"), (int, float)) and r["send_gap_ms"] > 0)
    metadata = next((r for r in records if r.get("type") == "session_metadata"), {})
    period = metadata.get("config", {}).get("mit", {}).get("period_ms")
    period = period if isinstance(period, (int, float)) and period > 0 else None
    mean_gap = sum(gaps) / len(gaps) if gaps else None
    failed = [r for r in records if r.get("type") in ("rpc_end", "mit_result") and r.get("result") != 1]
    return {
        "pv": {
            "samples": len(pv),
            "invalid_samples": sum(not r.get("valid") for r in pv),
            "send_gap_mean_ms": sum(pv_gaps)/len(pv_gaps) if pv_gaps else None,
            "send_gap_max_ms": max(pv_gaps) if pv_gaps else None,
            "send_gap_p99_ms": pv_gaps[max(0, (99*len(pv_gaps)+99)//100-1)] if pv_gaps else None,
            "effective_send_frequency_hz": 1000*len(pv_gaps)/sum(pv_gaps) if pv_gaps else None,
            "send_gaps_over_20ms": sum(g > 20 for g in pv_gaps),
            "max_tracking_error_rad": max(pv_errors) if pv_errors else None,
            "note": "Target vs synchronous reply; not time-aligned encoder truth or proof of controller interpolation.",
        },
        "mit_attempts_recorded": len(inputs),
        "mit_results_recorded": len(frames),
        "mit_acknowledged": sum(r.get("result") == 1 for r in frames),
        "unmatched_mit_attempts": max(0, len(inputs) - len(frames)),
        "first_frame_delays_ms": [r.get("since_mode_ack_ms") for r in frames if r.get("frame") == 1],
        "send_gap_max_ms": max(gaps) if gaps else None,
        "target_period_ms": period,
        "target_frequency_hz": 1000 / period if period else None,
        "send_gap_mean_ms": mean_gap,
        "effective_send_frequency_hz": 1000 / mean_gap if mean_gap else None,
        "send_gap_p99_ms": gaps[max(0, (99 * len(gaps) + 99) // 100 - 1)] if gaps else None,
        "send_gaps_over_2_periods": sum(g > 2 * period for g in gaps) if period else None,
        "send_gap_p95_ms": gaps[max(0, (95 * len(gaps) + 99) // 100 - 1)] if gaps else None,
        "sdk_failures": failed,
        "events": [r for r in records if r.get("type") == "event" and
                   any(s in r.get("action", "") for s in ("fault", "exit", "error", "stop", "jacobian"))],
        "notes": [r.get("note") for r in records if r.get("type") == "operator_note"],
        "interpretation": "SDK acknowledgements are not proof of execution or safe exit. "
                          "A -1 result alone cannot distinguish local validation, transport or controller rejection. "
                          "Host send gaps are not controller receive gaps; controller thresholds remain unknown.",
    }


def read_session(directory):
    records, warnings = [], []
    path = Path(directory) / "diagnostics.jsonl"
    if path.exists():
        with path.open(encoding="utf-8") as stream:
            for line_no, line in enumerate(stream, 1):
                try:
                    record = json.loads(line)
                    if not isinstance(record, dict):
                        raise ValueError("expected object")
                    records.append(record)
                except (ValueError, TypeError):
                    warnings.append(f"Malformed/incomplete diagnostics line {line_no}")
    else:
        warnings.append("No diagnostics.jsonl: old session; frame timing cannot be reconstructed.")
        events = Path(directory) / "events.csv"
        if events.exists():
            with events.open(encoding="utf-8", newline="") as stream:
                records.extend(dict(row, type="event") for row in csv.DictReader(stream))
    report = summarize(records)
    report["warnings"] = warnings
    report["session"] = str(Path(directory).resolve())
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    args = parser.parse_args()
    if not args.session.is_dir():
        parser.error("session must be an existing directory")
    print(json.dumps(read_session(args.session), ensure_ascii=False, indent=2))
