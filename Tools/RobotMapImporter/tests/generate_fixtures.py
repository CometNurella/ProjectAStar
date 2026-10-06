"""Deterministic multicolour drawing fixtures; these are synthetic, not real scans."""
from pathlib import Path
import json
import math

import numpy as np
from PIL import Image, ImageDraw, ImageFont


HERE = Path(__file__).resolve().parent
FIXTURES = HERE / "fixtures"
SIZE = 1200
GRID = 60


def font(size):
    for candidate in [Path(r"C:\Windows\Fonts\arialbd.ttf"), Path(r"C:\Windows\Fonts\DejaVuSans.ttf")]:
        if candidate.exists():
            return ImageFont.truetype(str(candidate), size)
    raise RuntimeError("A TrueType font is required for the OCR fixture.")


def centered_text(draw, rectangle, label, size=42):
    left, top, right, bottom = rectangle
    face = font(size)
    box = draw.textbbox((0, 0), label, font=face)
    x = (left + right - (box[2] - box[0])) / 2
    y = (top + bottom - (box[3] - box[1])) / 2 - box[1]
    draw.text((x, y), label, fill=(20, 20, 20), font=face)
    return [int(x + box[0]), int(y + box[1]), int(x + box[2]), int(y + box[3])]


def homography(source, target):
    rows, result = [], []
    for (x, y), (u, v) in zip(source, target):
        rows += [[x, y, 1, 0, 0, 0, -u*x, -u*y], [0, 0, 0, x, y, 1, -v*x, -v*y]]
        result += [u, v]
    h = np.append(np.linalg.solve(np.asarray(rows, dtype=float), np.asarray(result, dtype=float)), 1)
    return h.reshape(3, 3)


def write_fixture():
    FIXTURES.mkdir(parents=True, exist_ok=True)
    image = Image.new("RGB", (SIZE, SIZE), "white")
    draw = ImageDraw.Draw(image)
    # Closed outlines in two hues, and a filled concave object in a third hue.
    draw.rectangle((100, 120, 470, 330), outline=(25, 80, 190), width=7)
    blue_box = centered_text(draw, (100, 120, 470, 330), "OBSTACLE")
    draw.ellipse((690, 80, 1050, 410), outline=(20, 135, 60), width=7)
    green_box = centered_text(draw, (690, 80, 1050, 410), "OBSTACLE")
    concave = [(130, 530), (620, 530), (620, 700), (320, 700), (320, 1050), (130, 1050)]
    draw.polygon(concave, fill=(238, 135, 135), outline=(180, 25, 25), width=6)
    red_box = centered_text(draw, (130, 530, 620, 700), "OBSTACLE")
    # A multicolour map must not mark every coloured item as an obstacle.
    draw.rounded_rectangle((750, 700, 1070, 1010), radius=35, fill=(245, 194, 70), outline=(160, 110, 5), width=5)
    centered_text(draw, (750, 700, 1070, 1010), "PARK", 42)
    image.save(FIXTURES / "multicolour_scan.png")

    no_label = Image.new("RGB", (SIZE, SIZE), "white")
    no_label_draw = ImageDraw.Draw(no_label)
    no_label_draw.rectangle((300, 300, 900, 700), fill=(245, 194, 70), outline=(160, 110, 5), width=5)
    centered_text(no_label_draw, (300, 300, 900, 700), "PARK", 42)
    no_label.save(FIXTURES / "no_obstacle_label.png")

    open_shape = Image.new("RGB", (SIZE, SIZE), "white")
    open_draw = ImageDraw.Draw(open_shape)
    open_draw.rectangle((250, 300, 950, 850), outline=(25, 80, 190), width=7)
    open_draw.rectangle((500, 295, 650, 315), fill="white")
    centered_text(open_draw, (250, 300, 950, 850), "OBSTACLE", 42)
    open_shape.save(FIXTURES / "open_outline.png")

    # Deterministic uneven illumination and sensor noise, without a geometric warp.
    data = np.asarray(image, dtype=np.float64)
    yy, xx = np.mgrid[0:SIZE, 0:SIZE]
    illumination = 0.78 + 0.20 * (xx / (SIZE - 1)) - 0.08 * (yy / (SIZE - 1))
    rng = np.random.default_rng(1206)
    noisy = np.clip(data * illumination[:, :, None] + rng.normal(0, 1.2, data.shape), 0, 255).astype(np.uint8)
    Image.fromarray(noisy).save(FIXTURES / "multicolour_shadow.png")

    # Perspective photo simulation. These corners are provided to rectification.
    source_corners = [(0, 0), (SIZE-1, 0), (SIZE-1, SIZE-1), (0, SIZE-1)]
    photo_corners = [(160, 120), (1250, 65), (1330, 1280), (80, 1190)]
    h = homography(source_corners, photo_corners)
    inverse = np.linalg.inv(h)
    inverse /= inverse[2, 2]
    photo = image.transform((1420, 1360), Image.Transform.PERSPECTIVE,
        inverse.ravel()[:8].tolist(), resample=Image.Resampling.BICUBIC, fillcolor=(190, 190, 180))
    photo.save(FIXTURES / "multicolour_perspective.png")

    # Ground truth probes are deliberately away from thin boundary cells.
    def cell(pixel):
        return [int(pixel[0] // (SIZE / GRID)), GRID - 1 - int(pixel[1] // (SIZE / GRID))]

    probes = {
        "blue_outline_interior": {"cell": cell((280, 240)), "blocked": True},
        "green_outline_interior": {"cell": cell((850, 300)), "blocked": True},
        "red_concave_top": {"cell": cell((460, 630)), "blocked": True},
        "red_concave_leg": {"cell": cell((220, 880)), "blocked": True},
        "red_concave_free_notch": {"cell": cell((460, 880)), "blocked": False},
        "unlabelled_coloured_shape": {"cell": cell((900, 900)), "blocked": False},
        "white_corridor": {"cell": cell((600, 390)), "blocked": False},
    }
    manifest = {
        "description": "Synthetic printed-label drawing fixtures, not photographs of handwriting.",
        "grid_width": GRID, "grid_height": GRID, "cell_size": 0.5,
        "rectified_size": [SIZE, SIZE], "start_cell": [3, 3], "goal_cell": [55, 55],
        "label_boxes": [blue_box, green_box, red_box],
        "photo_corners_tl_tr_br_bl": photo_corners,
        "probes": probes,
    }
    (FIXTURES / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(json.dumps({"fixtures": str(FIXTURES), "probe_count": len(probes)}, indent=2))


if __name__ == "__main__":
    write_fixture()
