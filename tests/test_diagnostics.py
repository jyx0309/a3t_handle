import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location("analyzer", Path(__file__).parents[1] / "tools/analyze_session.py")
analyzer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(analyzer)


class DiagnosticsTest(unittest.TestCase):
    def test_pv_timing(self):
        report = analyzer.summarize([
            {"type": "pv_tracking", "send_gap_ms": gap, "valid": True, "max_error_rad": 0.002}
            for gap in [-1, 10, 10, 10]])["pv"]
        self.assertEqual(report["effective_send_frequency_hz"], 100)
        self.assertEqual(report["samples"], 4)
        self.assertEqual(report["invalid_samples"], 0)
        self.assertEqual(report["max_tracking_error_rad"], 0.002)
        self.assertIsNone(analyzer.summarize([])["pv"]["effective_send_frequency_hz"])

    def test_200hz_timing(self):
        report = analyzer.summarize([
            {"type": "session_metadata", "config": {"mit": {"period_ms": 5}}},
            *[{"type": "mit_result", "frame": i, "result": 1, "send_gap_ms": gap}
              for i, gap in [(1, 500), (2, 4), (3, 5), (4, 6), (5, 11)]]])
        self.assertEqual(report["target_frequency_hz"], 200)
        self.assertEqual(report["send_gap_mean_ms"], 6.5)
        self.assertAlmostEqual(report["effective_send_frequency_hz"], 1000 / 6.5)
        self.assertEqual(report["send_gap_p99_ms"], 11)
        self.assertEqual(report["send_gaps_over_2_periods"], 1)

    def test_first_frame_rejected(self):
        report = analyzer.summarize([
            {"type": "mit_input", "frame": 1},
            {"type": "mit_result", "frame": 1, "result": -1, "send_gap_ms": -1, "since_mode_ack_ms": 39},
            {"type": "event", "action": "fault", "detail": "mode invalid"},
        ])
        self.assertEqual(report["mit_acknowledged"], 0)
        self.assertEqual(report["first_frame_delays_ms"], [39])
        self.assertIsNone(report["send_gap_max_ms"])
        self.assertEqual(len(report["sdk_failures"]), 1)

    def test_missing_reply_and_empty(self):
        report = analyzer.summarize([{"type": "mit_input", "frame": 1}])
        self.assertEqual(report["unmatched_mit_attempts"], 1)
        self.assertEqual(analyzer.summarize([])["mit_acknowledged"], 0)

    def test_multiple_runs_exclude_first_gap(self):
        report = analyzer.summarize([
            {"type": "mit_result", "frame": i, "result": 1, "send_gap_ms": gap}
            for i, gap in [(1, -1), (2, 10), (3, 15), (1, -1)]])
        self.assertEqual(report["send_gap_max_ms"], 15)
        self.assertEqual(report["send_gap_p95_ms"], 15)


if __name__ == "__main__":
    unittest.main()
