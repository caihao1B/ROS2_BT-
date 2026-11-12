#!/usr/bin/env python3
"""Small, synthetic teaching examples. Standard library only; no ROS side effects.

These equations illustrate concepts, not the production 15-state estimator.
"""
import json
import math


def scalar_update(mean, variance, measurement, measurement_variance):
    gain = variance / (variance + measurement_variance)
    updated = mean + gain * (measurement - mean)
    # Joseph form, as used for numerical robustness in Kalman filters.
    covariance = (1 - gain) ** 2 * variance + gain ** 2 * measurement_variance
    return updated, covariance, gain


def velocity_demo(wheel_variance):
    mean, variance, _ = scalar_update(0.4, 0.04, 0.42, 0.01)
    return scalar_update(mean, variance, 0.7, wheel_variance)


def compose(a, b):
    """T_A_C = T_A_B * T_B_C, using SE(2) tuples (x, y, yaw radians)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1],
            a[1] + s * b[0] + c * b[1], a[2] + b[2])


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2])


def gnss_gate(dx, dy, sigma=0.04):
    # Simplified diagonal 2D innovation covariance, teaching units: m and m².
    variance = sigma ** 2 + 0.0025
    distance = math.hypot(dx, dy)
    nis = (dx ** 2 + dy ** 2) / variance
    return dict(distance_m=distance, nis=nis, distance_ok=distance <= 0.15,
                nis_ok=nis <= 9.21, accepted=distance <= 0.15 and nis <= 9.21)


def graph_demo(weight=1):
    """Anchor x0=0; measured increments 1.1,1.1; loop x2-x0=2.0.

    Minimize (x1-1.1)^2+(x2-x1-1.1)^2+weight*(x2-2)^2.
    H = [[2,-1],[-1,1+weight]], b = [0,1.1+2*weight].
    """
    x1 = (1.1 + 2 * weight) / (1 + 2 * weight)
    return x1, 2 * x1


def normal_information_min_eigenvalue(normals):
    """Minimum eigenvalue of the 2x2 sum of normal outer products.

    This models only translational normal-direction information, not yaw.
    """
    xx=sum(x*x for x,y in normals)
    xy=sum(x*y for x,y in normals)
    yy=sum(y*y for x,y in normals)
    return ((xx+yy)-math.sqrt((xx-yy)**2+4*xy*xy))/2


def main():
    normal, degraded = velocity_demo(0.0025), velocity_demo(4.0)
    assert abs(normal[0] - 0.7) < abs(degraded[0] - 0.7)
    assert abs(degraded[0] - 0.42) < abs(normal[0] - 0.42)

    local = (2.0, 1.0, 0.1)
    correction = (0.4, -0.2, math.radians(10))
    global_pose = compose(correction, local)
    recovered = compose(global_pose, inverse(local))
    assert all(abs(a-b) < 1e-12 for a,b in zip(recovered, correction))
    assert local == (2.0, 1.0, 0.1)  # map correction never mutates local odometry.

    gate = gnss_gate(0.16, 0.0)
    assert gate['nis_ok'] and not gate['distance_ok'] and not gate['accepted']
    assert gnss_gate(0.10, 0.0)['accepted']

    x1, x2 = graph_demo()
    assert abs(2*x1-x2) < 1e-12
    assert abs(-x1+2*x2-3.1) < 1e-12  # normal equations, not a frozen expected print.
    assert 2.0 < x2 < 2.2

    # Normal-direction information only, not a complete ICP Hessian:
    # two parallel wall normals -> diag(2,0); orthogonal normals -> diag(1,1).
    parallel_min_eigenvalue=normal_information_min_eigenvalue([(1,0),(1,0)])
    orthogonal_min_eigenvalue=normal_information_min_eigenvalue([(1,0),(0,1)])
    assert parallel_min_eigenvalue == 0 < orthogonal_min_eigenvalue
    diagonal=math.sqrt(0.5)
    assert abs(normal_information_min_eigenvalue([(diagonal,diagonal),(-diagonal,diagonal)])-orthogonal_min_eigenvalue)<1e-12

    report = dict(kind='SYNTHETIC_TEACHING_ONLY', checks_passed=5,
                  velocity=dict(normal_mps=normal[0], slip_downweighted_mps=degraded[0],
                                normal_wheel_gain=normal[2], degraded_wheel_gain=degraded[2]),
                  transform=dict(local_unchanged=local, global_pose=global_pose,
                                 recovered_map_odom=recovered),
                  gnss_16cm=gate,
                  graph=dict(before=[0,1.1,2.2], after=[0,x1,x2]),
                  normal_geometry=dict(parallel_min_eigenvalue=parallel_min_eigenvalue,
                                       orthogonal_min_eigenvalue=orthogonal_min_eigenvalue))
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print('PASS: 5 个教学检查通过；不代表生产阶段验收。')


if __name__ == '__main__':
    main()
