from pathlib import Path


DETECTOR = Path(__file__).resolve().parents[1] / "launch" / "detect.py"


def detector_source():
    return DETECTOR.read_text(encoding="utf-8")


def test_fused_topic_uses_the_epoch4_qat_conservative_int8_plan():
    source = detector_source()
    assert "MODEL_UNITREE_PLAN_PATH" in source
    assert '"model/fused_epoch4_qat_conservative_c/"' in source
    assert '"fused_epoch4_qat_conservative_c_int8.plan"' in source
    assert '"model/fused_epoch4/fused_epoch4_fp16.plan"' not in source
    assert "Loading unitree TensorRT engine" in source


def test_fused_engine_failure_falls_back_to_epoch4_pytorch():
    source = detector_source()
    assert "Falling back to unitree PyTorch checkpoint" in source
    assert "MODEL_UNITREE_CHECKPOINT_PATH" in source
    assert "UnitreeCAModel" in source


def test_kitti_and_simulation_keep_existing_tensorrt_plan():
    source = detector_source()
    assert 'MODEL_PLAN_PATH = "/home/winter24/ros2_ws/src/Ros2-Autonomous-Hmi/model/model.plan"' in source
    assert 'self.load_model("base_demo_kitti.json", MODEL_PLAN_PATH, "kitti")' in source
