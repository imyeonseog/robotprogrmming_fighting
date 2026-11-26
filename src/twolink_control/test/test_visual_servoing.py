import math

from twolink_control.visual_servoing import (
    forward_kinematics,
    inverse_kinematics,
    marker_yaw_error,
)


def test_fk_ik_roundtrip():
    l1, l2 = 0.143, 0.120
    for q1_orig, q2_orig in [(0.0, 1.57), (0.5, 1.2), (-0.4, 0.8), (0.2, 2.0)]:
        x, y, _ = forward_kinematics(q1_orig, q2_orig, l1, l2)
        q1_rec, q2_rec = inverse_kinematics(x, y, l1, l2, elbow_sign=1.0)
        x_rec, y_rec, _ = forward_kinematics(q1_rec, q2_rec, l1, l2)
        assert math.isclose(x, x_rec, abs_tol=1e-5)
        assert math.isclose(y, y_rec, abs_tol=1e-5)


def test_ik_clamps_out_of_workspace():
    l1, l2 = 0.143, 0.120
    q1, q2 = inverse_kinematics(10.0, 0.0, l1, l2, elbow_sign=1.0)
    x, y, _ = forward_kinematics(q1, q2, l1, l2)
    assert math.hypot(x, y) <= l1 + l2 + 1e-6


def test_marker_yaw_error_head_on():
    # Rotation of 180 deg around X maps marker +Z to -Z_cam (head-on facing camera)
    err = marker_yaw_error(1.0, 0.0, 0.0, 0.0)
    assert math.isclose(err, 0.0, abs_tol=1e-6)
