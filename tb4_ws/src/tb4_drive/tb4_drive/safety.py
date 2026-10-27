"""Pure-python helpers (no ROS imports) so the logic can be unit-tested anywhere."""
import math


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def ramp(current, target, max_delta):
    """Move `current` toward `target` by at most `max_delta`."""
    return current + clamp(target - current, -max_delta, max_delta)


def sector_min(ranges, angle_min, angle_inc, range_min, range_max,
               center, half_width, frame_yaw=0.0):
    """Smallest valid range inside a sector.

    `center` / `half_width` are expressed in the robot frame (0 = straight ahead, CCW+).
    `frame_yaw` is the yaw of the lidar frame in the robot frame, so a beam at scan angle `a`
    points at robot angle `a + frame_yaw`. Returns math.inf if no valid beam is in the sector.
    """
    best = math.inf
    for i, r in enumerate(ranges):
        if not math.isfinite(r) or r < range_min or r > range_max:
            continue
        a = angle_min + i * angle_inc + frame_yaw
        d = math.atan2(math.sin(a - center), math.cos(a - center))  # wrapped difference
        if abs(d) <= half_width and r < best:
            best = r
    return best


def safe_linear(v, front_dist, rear_dist, stop_dist, slow_dist):
    """Scale / block the commanded linear speed according to obstacle distance in the travel direction."""
    if v > 0.0:
        d = front_dist
    elif v < 0.0:
        d = rear_dist
    else:
        return 0.0
    if d <= stop_dist:
        return 0.0
    if d >= slow_dist:
        return v
    return v * (d - stop_dist) / (slow_dist - stop_dist)
