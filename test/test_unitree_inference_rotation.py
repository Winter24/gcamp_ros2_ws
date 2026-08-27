from pathlib import Path

DETECT_PATH = Path(__file__).parents[1] / "launch" / "detect.py"


def test_unitree_inference_maps_input_120_degrees_and_inverse_maps_predictions():
    source = DETECT_PATH.read_text(encoding="utf-8")

    assert "UNITREE_INFERENCE_YAW_RAD = math.radians(120.0)" in source
    assert "rotate_points_z(raw_points, UNITREE_INFERENCE_YAW_RAD)" in source
    assert "rotate_detection_boxes_z(boxes, -UNITREE_INFERENCE_YAW_RAD)" in source


def test_fused_points_uses_selected_sq2_axis_fixed_epoch_4_checkpoint():
    source = DETECT_PATH.read_text(encoding="utf-8")

    assert '"model/finetune/sq2_axis_fixed_balanced_epoch4_checkpoint_pytorch_fp32"' in source


def test_unitree_yaw_uses_inverse_mapped_value_without_extra_offset():
    source = DETECT_PATH.read_text(encoding="utf-8")

    assert "return float(model_yaw) if self.dataset_type == 'unitree' else -float(model_yaw)" in source
    assert "UNITREE_YAW_OFFSET_DEG = 0.0" in source


def test_unitree_boxes_do_not_apply_a_second_axis_offset():
    source = DETECT_PATH.read_text(encoding="utf-8")

    assert "UNITREE_YAW_OFFSET_DEG = 0.0" in source


def test_unitree_bbox_size_scale_is_zero_point_seven_five():
    source = DETECT_PATH.read_text(encoding="utf-8")

    assert "UNITREE_SIZE_SCALE     = 0.75" in source


def test_unitree_runtime_roi_is_shifted_back_thirty_metres():
    config = (DETECT_PATH.parent / "base_demo_unitree.json").read_text(encoding="utf-8")

    assert '"x_min": -30.0' in config
    assert '"x_max": 40.4' in config


def test_each_source_uses_its_own_score_threshold():
    source = DETECT_PATH.read_text(encoding="utf-8")

    assert "SIMULATION_SCORE_THRESHOLD = 0.45" in source
    assert "UNITREE_SCORE_THRESHOLD = 0.5" in source
    assert "DEFAULT_SCORE_THRESHOLD = 0.3" in source
    assert "self.current_topic == '/points_raw'" in source
    assert "self.dataset_type == 'unitree'" in source
