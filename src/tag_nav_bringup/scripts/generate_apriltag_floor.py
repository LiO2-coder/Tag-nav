#!/usr/bin/env python3
"""Generate the AprilTag atlas and the single-visual Gazebo floor mesh."""

from argparse import ArgumentParser
from pathlib import Path

from PIL import Image


FAMILY_TAG_COUNT = 587
USED_TAG_COUNT = 475
GRID_COLUMNS = 19
GRID_ROWS = 25
TILE_PIXELS = 256
SOURCE_PIXELS = 816
DETECTABLE_CODE_PIXELS = 544
DETECTABLE_CODE_SIZE_METERS = 0.10
STICKER_SIZE_METERS = (
    DETECTABLE_CODE_SIZE_METERS * SOURCE_PIXELS / DETECTABLE_CODE_PIXELS
)
MIN_X = -9
MAX_Y = 12


def parse_args():
    package_root = Path(__file__).resolve().parents[1]
    parser = ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input-dir",
        type=Path,
        default=package_root / "worlds/assets/texture",
        help="Directory containing tag36h11_<id>.png files.",
    )
    parser.add_argument(
        "--model-dir",
        type=Path,
        default=package_root / "worlds/models/apriltag_floor",
        help="Output directory for the Gazebo model assets.",
    )
    return parser.parse_args()


def tag_position(tag_id):
    row, column = divmod(tag_id, GRID_COLUMNS)
    return MIN_X + column, MAX_Y - row


def load_source_images(input_dir):
    images = []
    missing = []
    invalid = []

    for tag_id in range(FAMILY_TAG_COUNT):
        path = input_dir / f"tag36h11_{tag_id}.png"
        if not path.is_file():
            missing.append(path.name)
            continue

        with Image.open(path) as source:
            if source.size != (SOURCE_PIXELS, SOURCE_PIXELS) or source.mode != "RGB":
                invalid.append(f"{path.name}: size={source.size}, mode={source.mode}")
            if tag_id < USED_TAG_COUNT:
                images.append(source.copy())

    if missing or invalid:
        details = []
        if missing:
            details.append("missing: " + ", ".join(missing))
        if invalid:
            details.append("invalid: " + "; ".join(invalid))
        raise RuntimeError("AprilTag input validation failed: " + " | ".join(details))

    if len(images) != USED_TAG_COUNT:
        raise RuntimeError(f"Expected {USED_TAG_COUNT} used images, found {len(images)}")
    return images


def write_atlas(images, output_path):
    atlas = Image.new(
        "RGB",
        (GRID_COLUMNS * TILE_PIXELS, GRID_ROWS * TILE_PIXELS),
        (0, 0, 0),
    )
    resampling = getattr(Image, "Resampling", Image).NEAREST

    for tag_id, source in enumerate(images):
        resized = source.resize((TILE_PIXELS, TILE_PIXELS), resampling)
        row, column = divmod(tag_id, GRID_COLUMNS)
        atlas.paste(resized, (column * TILE_PIXELS, row * TILE_PIXELS))

    output_path.parent.mkdir(parents=True, exist_ok=True)
    atlas.save(output_path, format="PNG", optimize=True, compress_level=9)


def write_obj(output_path):
    lines = [
        "# Generated AprilTag floor mesh.",
        "# Coordinates use +X to the right and +Y upward in the map view.",
        "mtllib apriltag_grid.mtl",
        "o apriltag_grid",
        "s off",
        "vn 0.0 0.0 1.0",
    ]
    vertices = []
    uvs = []
    faces = []

    half_size = STICKER_SIZE_METERS / 2.0
    for tag_id in range(USED_TAG_COUNT):
        x, y = tag_position(tag_id)
        row, column = divmod(tag_id, GRID_COLUMNS)
        x0, x1 = x - half_size, x + half_size
        y0, y1 = y - half_size, y + half_size

        # OBJ v coordinates start at the bottom, while the atlas rows start at the top.
        u0 = column / GRID_COLUMNS
        u1 = (column + 1) / GRID_COLUMNS
        v0 = 1.0 - (row + 1) / GRID_ROWS
        v1 = 1.0 - row / GRID_ROWS

        vertex_start = len(vertices) + 1
        uv_start = len(uvs) + 1
        vertices.extend(((x0, y0, 0.0), (x1, y0, 0.0), (x1, y1, 0.0), (x0, y1, 0.0)))
        uvs.extend(((u0, v0), (u1, v0), (u1, v1), (u0, v1)))
        faces.append(
            "f "
            + " ".join(
                f"{vertex_start + offset}/{uv_start + offset}/1" for offset in range(4)
            )
        )

    lines.extend(f"v {x:.6f} {y:.6f} {z:.6f}" for x, y, z in vertices)
    lines.extend(f"vt {u:.9f} {v:.9f}" for u, v in uvs)
    lines.append("usemtl apriltag_atlas")
    lines.extend(faces)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text("\n".join(lines) + "\n", encoding="ascii")


def write_mtl(output_path):
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        """# Material name is also used by the SDF visual material.
newmtl apriltag_atlas
Ka 1.0 1.0 1.0
Kd 1.0 1.0 1.0
Ks 0.0 0.0 0.0
d 1.0
illum 1
""",
        encoding="ascii",
    )


def main():
    args = parse_args()
    images = load_source_images(args.input_dir)

    atlas_path = args.model_dir / "materials/textures/apriltag_atlas.png"
    mesh_path = args.model_dir / "meshes/apriltag_grid.obj"
    mtl_path = args.model_dir / "meshes/apriltag_grid.mtl"
    write_atlas(images, atlas_path)
    write_obj(mesh_path)
    write_mtl(mtl_path)

    print(f"Validated {FAMILY_TAG_COUNT} source tags")
    print(f"Generated {USED_TAG_COUNT} used tags")
    print(
        "Detectable code: "
        f"{DETECTABLE_CODE_SIZE_METERS:.3f}m "
        f"({DETECTABLE_CODE_PIXELS} of {SOURCE_PIXELS} source pixels)"
    )
    print(f"Full printed sticker: {STICKER_SIZE_METERS:.3f}m x {STICKER_SIZE_METERS:.3f}m")
    print(f"Atlas: {atlas_path} ({GRID_COLUMNS * TILE_PIXELS}x{GRID_ROWS * TILE_PIXELS})")
    print(f"Mesh: {mesh_path}")
    print(f"Material: {mtl_path}")


if __name__ == "__main__":
    main()
