from pathlib import Path


DETECTOR = Path(__file__).resolve().parents[1] / "launch" / "detect.py"


def test_detector_total_latency_includes_worker_queue_wait():
    source = DETECTOR.read_text(encoding="utf-8")

    assert "time.perf_counter()" in source
    assert "self._latest_msg = (msg, received_at)" in source
    assert "queue_wait_ms" in source
    assert '"detector_total_latency_ms"' in source


def test_detector_keeps_processing_latency_for_backward_compatibility():
    source = DETECTOR.read_text(encoding="utf-8")

    assert '"process_time_ms"' in source


def test_detector_reports_input_pipeline_and_inference_fps_separately():
    source = DETECTOR.read_text(encoding="utf-8")

    assert '"input_fps"' in source
    assert '"pipeline_fps"' in source
    assert '"inference_fps"' in source
    assert '"fps": float(display_fps)' in source
