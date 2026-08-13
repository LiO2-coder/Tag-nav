#!/usr/bin/env python3
"""Generate a map_server occupancy map from the factory Gazebo world."""

from __future__ import print_function

import argparse
import math
import os
import xml.etree.ElementTree as ET


RESOLUTION = 0.05
# These bounds surround the physical outer-wall collision volumes by one pixel.
X_MIN = -10.15
X_MAX = 10.15
Y_MIN = -12.65
Y_MAX = 12.65
FREE = 254
OCCUPIED = 0
UNKNOWN = 205


def pose_values(element):
    """Return SDF pose values, defaulting to the identity pose."""
    pose = element.find("pose")
    if pose is None or not pose.text:
        return (0.0,) * 6
    values = tuple(float(value) for value in pose.text.split())
    if len(values) != 6:
        raise ValueError("SDF pose must contain six values")
    return values


def collision_shapes(world):
    """Read top-level static collision geometry and project it onto the floor."""
    for model in world.findall("model"):
        if model.findtext("static") != "1":
            continue

        model_x, model_y, _, _, _, model_yaw = pose_values(model)
        for link in model.findall("link"):
            link_x, link_y, _, _, _, link_yaw = pose_values(link)
            for collision in link.findall("collision"):
                collision_x, collision_y, _, _, _, collision_yaw = pose_values(collision)
                yaw = model_yaw + link_yaw + collision_yaw
                cos_yaw = math.cos(model_yaw + link_yaw)
                sin_yaw = math.sin(model_yaw + link_yaw)
                x = model_x + cos_yaw * (link_x + collision_x) - sin_yaw * (link_y + collision_y)
                y = model_y + sin_yaw * (link_x + collision_x) + cos_yaw * (link_y + collision_y)
                geometry = collision.find("geometry")
                if geometry is None:
                    continue

                box = geometry.find("box/size")
                if box is not None and box.text:
                    width, length, _ = (float(value) for value in box.text.split())
                    yield ("box", x, y, yaw, width, length)
                    continue

                cylinder = geometry.find("cylinder/radius")
                if cylinder is not None and cylinder.text:
                    yield ("cylinder", x, y, float(cylinder.text))


def pixel_range(minimum, maximum, origin, count):
    start = max(0, int(math.floor((minimum - origin) / RESOLUTION)))
    end = min(count - 1, int(math.ceil((maximum - origin) / RESOLUTION)) - 1)
    return range(start, end + 1)


def mark_box(image, width, height, center_x, center_y, yaw, box_width, box_length):
    half_width = box_width / 2.0
    half_length = box_length / 2.0
    extent_x = abs(half_width * math.cos(yaw)) + abs(half_length * math.sin(yaw))
    extent_y = abs(half_width * math.sin(yaw)) + abs(half_length * math.cos(yaw))
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)

    for grid_y in pixel_range(center_y - extent_y, center_y + extent_y, Y_MIN, height):
        world_y = Y_MIN + (grid_y + 0.5) * RESOLUTION
        for grid_x in pixel_range(center_x - extent_x, center_x + extent_x, X_MIN, width):
            world_x = X_MIN + (grid_x + 0.5) * RESOLUTION
            local_x = cos_yaw * (world_x - center_x) + sin_yaw * (world_y - center_y)
            local_y = -sin_yaw * (world_x - center_x) + cos_yaw * (world_y - center_y)
            if abs(local_x) <= half_width and abs(local_y) <= half_length:
                image[height - 1 - grid_y][grid_x] = OCCUPIED


def mark_cylinder(image, width, height, center_x, center_y, radius):
    radius_squared = radius * radius
    for grid_y in pixel_range(center_y - radius, center_y + radius, Y_MIN, height):
        world_y = Y_MIN + (grid_y + 0.5) * RESOLUTION
        for grid_x in pixel_range(center_x - radius, center_x + radius, X_MIN, width):
            world_x = X_MIN + (grid_x + 0.5) * RESOLUTION
            if (world_x - center_x) ** 2 + (world_y - center_y) ** 2 <= radius_squared:
                image[height - 1 - grid_y][grid_x] = OCCUPIED


def generate(world_path, output_directory):
    root = ET.parse(world_path).getroot()
    world = root.find("world")
    if world is None:
        raise ValueError("No <world> element found in {}".format(world_path))

    width = int(round((X_MAX - X_MIN) / RESOLUTION))
    height = int(round((Y_MAX - Y_MIN) / RESOLUTION))
    # The known region is the inside of the enclosing factory walls. Exterior
    # pixels remain unknown, which keeps map_server clients inside the arena.
    image = [[UNKNOWN] * width for _ in range(height)]
    for grid_y in pixel_range(-12.35, 12.35, Y_MIN, height):
        for grid_x in pixel_range(-9.85, 9.85, X_MIN, width):
            image[height - 1 - grid_y][grid_x] = FREE

    for shape in collision_shapes(world):
        if shape[0] == "box":
            mark_box(image, width, height, *shape[1:])
        elif shape[0] == "cylinder":
            mark_cylinder(image, width, height, *shape[1:])

    os.makedirs(output_directory, exist_ok=True)
    pgm_path = os.path.join(output_directory, "factory_map.pgm")
    yaml_path = os.path.join(output_directory, "factory_map.yaml")
    with open(pgm_path, "wb") as pgm_file:
        pgm_file.write("P5\n{} {}\n255\n".format(width, height).encode("ascii"))
        for row in image:
            pgm_file.write(bytearray(row))

    with open(yaml_path, "w") as yaml_file:
        yaml_file.write(
            "image: factory_map.pgm\n"
            "resolution: {:.2f}\n"
            "origin: [{:.2f}, {:.2f}, 0.0]\n"
            "negate: 0\n"
            "occupied_thresh: 0.65\n"
            "free_thresh: 0.196\n".format(RESOLUTION, X_MIN, Y_MIN)
        )

    print("Generated {} ({} x {}, {:.2f} m/pixel)".format(pgm_path, width, height, RESOLUTION))


def main():
    package_directory = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--world",
        default=os.path.join(package_directory, "worlds", "factory.world"),
        help="Gazebo SDF world to rasterize",
    )
    parser.add_argument(
        "--output-dir",
        default=os.path.join(package_directory, "maps"),
        help="Directory for factory_map.pgm and factory_map.yaml",
    )
    arguments = parser.parse_args()
    generate(arguments.world, arguments.output_dir)


if __name__ == "__main__":
    main()
