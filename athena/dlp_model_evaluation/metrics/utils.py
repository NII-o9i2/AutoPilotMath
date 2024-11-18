import math
from typing import Tuple
from common.type import *


def build_polygon_by_pos_and_heading(geometry_center_x, geometry_center_y, yaw, ego_width=2.1, ego_length=4.886):
    polygon_points = []
    half_width = 0.5 * ego_width
    half_length = 0.5 * ego_length
    dist_center_to_corner = math.sqrt(half_width ** 2 + half_length ** 2)
    angel_center_to_corner = math.atan(half_width / half_length)

    left_front_corner_x = geometry_center_x + \
        dist_center_to_corner * math.cos(angel_center_to_corner + yaw)
    left_front_corner_y = geometry_center_y + \
        dist_center_to_corner * math.sin(angel_center_to_corner + yaw)

    right_front_corner_x = geometry_center_x + \
        dist_center_to_corner * math.cos(angel_center_to_corner - yaw)
    right_front_corner_y = geometry_center_y - \
        dist_center_to_corner * math.sin(angel_center_to_corner - yaw)

    left_rear_corner_x = geometry_center_x - \
        dist_center_to_corner * math.cos(angel_center_to_corner - yaw)
    left_rear_corner_y = geometry_center_y + \
        dist_center_to_corner * math.sin(angel_center_to_corner - yaw)

    right_rear_corner_x = geometry_center_x - \
        dist_center_to_corner * math.cos(angel_center_to_corner + yaw)
    right_rear_corner_y = geometry_center_y - \
        dist_center_to_corner * math.sin(angel_center_to_corner + yaw)

    polygon_points.append((left_front_corner_x, left_front_corner_y))
    polygon_points.append((right_front_corner_x, right_front_corner_y))
    polygon_points.append((right_rear_corner_x, right_rear_corner_y))
    polygon_points.append((left_rear_corner_x, left_rear_corner_y))
    return polygon_points


def transform(x0, y0, theta0, x_prime, y_prime, theta_prime):
    cos_theta0 = math.cos(theta0)
    sin_theta0 = math.sin(theta0)
    x = x0 + (x_prime * cos_theta0) - (y_prime * sin_theta0)
    y = y0 + (x_prime * sin_theta0) + (y_prime * cos_theta0)
    theta = theta0 + theta_prime
    return x, y, theta


def points_to_segments(points_list):
    points = [Point2D(x, y) for x, y in points_list]
    segments = [Segment(points[i], points[i+1])
                for i in range(len(points) - 1)]
    return segments


def cross_product(p1, p2, p3):
    return (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x)


def intersects(seg1, seg2):
    # rapid_rejection
    if (max(seg1.start.x, seg1.end.x) < min(seg2.start.x, seg2.end.x) or
        max(seg1.start.y, seg1.end.y) < min(seg2.start.y, seg2.end.y) or
        max(seg2.start.x, seg2.end.x) < min(seg1.start.x, seg1.end.x) or
            max(seg2.start.y, seg2.end.y) < min(seg1.start.y, seg1.end.y)):
        return False
    # cross
    if (cross_product(seg2.start, seg2.end, seg1.start) * cross_product(seg2.start, seg2.end, seg1.end) > 0 or
            cross_product(seg1.start, seg1.end, seg2.start) * cross_product(seg1.start, seg1.end, seg2.end) > 0):
        return False

    return True


def find_closest_distance(list1, list2):
    min_distance = float('inf')
    closest_pair = None

    for p1 in list1:
        for p2 in list2:
            distance = math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)
            if distance < min_distance:
                min_distance = distance
                closest_pair = (p1, p2)

    return min_distance, closest_pair


def is_intersect(p1: Tuple[float, float], p2: Tuple[float, float],
                 p3: Tuple[float, float], p4: Tuple[float, float]) -> bool:
    def cross_product(v1, v2) -> float:
        """计算向量 v1 和 v2 的叉积"""
        return v1[0] * v2[1] - v1[1] * v2[0]

    def direction(p: Tuple[float, float],
                  q: Tuple[float, float],
                  r: Tuple[float, float]) -> float:
        """计算向量 pq 和 pr 的方向 (叉积结果)"""
        return cross_product((r[0] - p[0], r[1] - p[1]),
                             (q[0] - p[0], q[1] - p[1]))

    def on_segment(p: Tuple[float, float],
                   q: Tuple[float, float],
                   r: Tuple[float, float]) -> bool:
        """判断点 r 是否在线段 pq 上（并且 r 在 pq 范围内）"""
        return (min(p[0], q[0]) <= r[0] <= max(p[0], q[0]) and
                min(p[1], q[1]) <= r[1] <= max(p[1], q[1]))

    # 计算四组方向（两条线段的方向组合）
    d1 = direction(p3, p4, p1)
    d2 = direction(p3, p4, p2)
    d3 = direction(p1, p2, p3)
    d4 = direction(p1, p2, p4)

    # 1. 判断是否一般相交 (方向符号不同)
    if d1 * d2 < 0 and d3 * d4 < 0:
        return True

    # 2. 特殊情况：某点在线段上（处理共线情况）
    if d1 == 0 and on_segment(p3, p4, p1):
        return True
    if d2 == 0 and on_segment(p3, p4, p2):
        return True
    if d3 == 0 and on_segment(p1, p2, p3):
        return True
    if d4 == 0 and on_segment(p1, p2, p4):
        return True

    # 3. 其他情况：不相交
    return False
