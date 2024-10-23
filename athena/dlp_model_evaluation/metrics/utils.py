import math


def build_polygon_by_pos_and_heading(geometry_center_x, geometry_center_y, yaw, ego_width=2.1, ego_length=4.886):
    polygon_points = []
    half_width = 0.5 * ego_width
    half_length = 0.5 * ego_length
    dist_center_to_corner = math.sqrt(half_width ** 2 + half_length ** 2)
    angel_center_to_corner = math.atan(half_width / half_length)

    left_front_corner_x = geometry_center_x + dist_center_to_corner * math.cos(angel_center_to_corner + yaw)
    left_front_corner_y = geometry_center_y + dist_center_to_corner * math.sin(angel_center_to_corner + yaw)

    right_front_corner_x = geometry_center_x + dist_center_to_corner * math.cos(angel_center_to_corner - yaw)
    right_front_corner_y = geometry_center_y - dist_center_to_corner * math.sin(angel_center_to_corner - yaw)

    left_rear_corner_x = geometry_center_x - dist_center_to_corner * math.cos(angel_center_to_corner - yaw)
    left_rear_corner_y = geometry_center_y + dist_center_to_corner * math.sin(angel_center_to_corner - yaw)

    right_rear_corner_x = geometry_center_x - dist_center_to_corner * math.cos(angel_center_to_corner + yaw)
    right_rear_corner_y = geometry_center_y - dist_center_to_corner * math.sin(angel_center_to_corner + yaw)

    polygon_points.append((left_front_corner_x, left_front_corner_y))
    polygon_points.append((right_front_corner_x, right_front_corner_y))
    polygon_points.append((right_rear_corner_x, right_rear_corner_y))
    polygon_points.append((left_rear_corner_x, left_rear_corner_y))
    return polygon_points
