import math

from tb4_drive.safety import clamp, ramp, safe_linear, sector_min


def test_clamp_ramp():
    assert clamp(5, -1, 1) == 1
    assert ramp(0.0, 1.0, 0.1) == 0.1
    assert ramp(0.5, 0.0, 0.1) == 0.4
    assert ramp(0.0, 0.05, 0.1) == 0.05


def _scan(n=360):
    return [5.0] * n, -math.pi, 2 * math.pi / n


def test_sector_front_with_lidar_yaw():
    ranges, amin, inc = _scan()
    # lidar mounted yaw +90 deg: robot-forward is scan angle -90 deg
    i = round((-math.pi / 2 - amin) / inc)
    ranges[i] = 0.5
    front = sector_min(ranges, amin, inc, 0.1, 12.0, 0.0, 0.3, frame_yaw=math.pi / 2)
    rear = sector_min(ranges, amin, inc, 0.1, 12.0, math.pi, 0.3, frame_yaw=math.pi / 2)
    assert abs(front - 0.5) < 1e-9
    assert rear == 5.0


def test_sector_ignores_invalid():
    ranges = [math.inf, math.nan, 0.01, 20.0]
    assert sector_min(ranges, 0.0, 0.0, 0.1, 12.0, 0.0, 1.0) == math.inf


def test_sector_wraparound():
    ranges, amin, inc = _scan()
    ranges[0] = 0.4  # scan angle -pi == robot rear when yaw=0
    assert sector_min(ranges, amin, inc, 0.1, 12.0, math.pi, 0.1) == 0.4


def test_safe_linear():
    assert safe_linear(0.3, 0.2, 5.0, 0.35, 0.8) == 0.0          # blocked forward
    assert safe_linear(-0.3, 0.2, 5.0, 0.35, 0.8) == -0.3        # reversing is fine
    assert safe_linear(-0.3, 5.0, 0.2, 0.35, 0.8) == 0.0         # blocked behind
    assert safe_linear(0.3, 5.0, 5.0, 0.35, 0.8) == 0.3
    v = safe_linear(0.4, 0.575, 5.0, 0.35, 0.8)                  # halfway through slow zone
    assert abs(v - 0.2) < 1e-9
    assert safe_linear(0.0, 0.0, 0.0, 0.35, 0.8) == 0.0
