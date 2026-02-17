# trajectory.py
import math

def generate_waypoints(path_nodes, nodes):
    return [nodes[name] for name in path_nodes]


def catmull_rom_spline(points, samples_per_segment=20):
    if len(points) < 2:
        return points[:]

    if len(points) == 2:
        p0, p1 = points
        return [
            (
                p0[0] + (p1[0] - p0[0]) * t,
                p0[1] + (p1[1] - p0[1]) * t,
            )
            for t in [i / samples_per_segment for i in range(samples_per_segment + 1)]
        ]

    spline_points = []
    n = len(points)

    for i in range(n - 1):
        p0 = points[i - 1] if i - 1 >= 0 else points[i]
        p1 = points[i]
        p2 = points[i + 1]
        p3 = points[i + 2] if i + 2 < n else points[i + 1]

        for j in range(samples_per_segment + 1):
            t = j / samples_per_segment
            t2 = t * t
            t3 = t2 * t

            x = 0.5 * (
                (2 * p1[0])
                + (-p0[0] + p2[0]) * t
                + (2 * p0[0] - 5 * p1[0] + 4 * p2[0] - p3[0]) * t2
                + (-p0[0] + 3 * p1[0] - 3 * p2[0] + p3[0]) * t3
            )
            y = 0.5 * (
                (2 * p1[1])
                + (-p0[1] + p2[1]) * t
                + (2 * p0[1] - 5 * p1[1] + 4 * p2[1] - p3[1]) * t2
                + (-p0[1] + 3 * p1[1] - 3 * p2[1] + p3[1]) * t3
            )

            if i > 0 and j == 0:
                continue
            spline_points.append((x, y))

    return spline_points


def compute_cumulative_distances(points):
    distances = [0.0]
    total = 0.0
    for i in range(1, len(points)):
        dx = points[i][0] - points[i - 1][0]
        dy = points[i][1] - points[i - 1][1]
        total += math.hypot(dx, dy)
        distances.append(total)
    return distances


def generate_trapezoidal_profile(total_length, v_max, a_max, dt):
    if total_length <= 0.0:
        return [(0.0, 0.0, 0.0)]

    d_accel = (v_max * v_max) / (2 * a_max)

    if 2 * d_accel > total_length:
        v_peak = math.sqrt(total_length * a_max)
        t_accel = v_peak / a_max
        t_flat = 0.0
        t_total = 2 * t_accel
    else:
        v_peak = v_max
        t_accel = v_max / a_max
        d_flat = total_length - 2 * d_accel
        t_flat = d_flat / v_max
        t_total = 2 * t_accel + t_flat

    profile = []
    t = 0.0
    while t < t_total + 1e-9:
        if t <= t_accel:
            v = a_max * t
            s = 0.5 * a_max * t * t
        elif t <= t_accel + t_flat:
            v = v_peak
            s = d_accel + v_peak * (t - t_accel)
        else:
            t_dec = t - t_accel - t_flat
            v = v_peak - a_max * t_dec
            s = d_accel + v_peak * t_flat + v_peak * t_dec - 0.5 * a_max * t_dec * t_dec

        if s > total_length:
            s = total_length
            v = 0.0
        profile.append((t, s, v))
        t += dt

    if profile[-1][1] < total_length:
        profile.append((t_total, total_length, 0.0))

    return profile


def interpolate_point(points, cum_dist, s):
    if s <= 0.0:
        return points[0], 0
    if s >= cum_dist[-1]:
        return points[-1], len(points) - 2

    for i in range(1, len(cum_dist)):
        if cum_dist[i] >= s:
            s0 = cum_dist[i - 1]
            s1 = cum_dist[i]
            ratio = (s - s0) / (s1 - s0) if s1 > s0 else 0.0
            x = points[i - 1][0] + (points[i][0] - points[i - 1][0]) * ratio
            y = points[i - 1][1] + (points[i][1] - points[i - 1][1]) * ratio
            return (x, y), i - 1

    return points[-1], len(points) - 2


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def generate_trajectory(
    path_nodes,
    nodes,
    v_max=1.0,
    a_max=0.5,
    dt=0.1,
    samples_per_segment=20,
):
    waypoints = generate_waypoints(path_nodes, nodes)
    if len(waypoints) < 2:
        return []

    spline_points = catmull_rom_spline(waypoints, samples_per_segment)
    cum_dist = compute_cumulative_distances(spline_points)
    total_length = cum_dist[-1]

    profile = generate_trapezoidal_profile(total_length, v_max, a_max, dt)

    trajectory = []
    prev_heading = None
    for t, s, v in profile:
        point, seg_idx = interpolate_point(spline_points, cum_dist, s)

        if seg_idx < len(spline_points) - 1:
            dx = spline_points[seg_idx + 1][0] - spline_points[seg_idx][0]
            dy = spline_points[seg_idx + 1][1] - spline_points[seg_idx][1]
        else:
            dx = spline_points[seg_idx][0] - spline_points[seg_idx - 1][0]
            dy = spline_points[seg_idx][1] - spline_points[seg_idx - 1][1]

        heading = math.atan2(dy, dx)
        if prev_heading is None:
            omega = 0.0
        else:
            omega = normalize_angle(heading - prev_heading) / dt

        trajectory.append(
            {
                "t": t,
                "x": point[0],
                "y": point[1],
                "v": v,
                "heading": heading,
                "omega": omega,
            }
        )
        prev_heading = heading

    return trajectory
