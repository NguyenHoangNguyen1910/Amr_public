"""Read standard occupancy-map PGM/YAML artifacts."""

from __future__ import annotations

import math
import re
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True, slots=True)
class OccupancyMap:
    """A validated occupancy map in planner cell encoding.

    Cells use the planner convention: ``0`` free, ``100`` occupied and
    ``255`` unknown. The source files remain standard PGM/YAML artifacts.
    """

    width: int
    height: int
    resolution_m: float
    origin_east_m: float
    origin_north_m: float
    cells: bytes
    sequence: int = 0

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError("Occupancy map dimensions must be positive")
        if not math.isfinite(self.resolution_m) or self.resolution_m <= 0:
            raise ValueError("Occupancy map resolution must be positive")
        if not math.isfinite(self.origin_east_m) or not math.isfinite(self.origin_north_m):
            raise ValueError("Occupancy map origin must be finite")
        if len(self.cells) != self.width * self.height:
            raise ValueError("Occupancy map payload length does not match dimensions")


def read_pgm(path: str | Path) -> tuple[int, int, bytes]:
    """Read one binary PGM (P5) image without external image dependencies."""

    data = Path(path).read_bytes()
    if not data.startswith(b"P5"):
        raise ValueError(f"{path} is not a binary PGM (P5)")

    tokens: list[bytes] = []
    index = 2
    while len(tokens) < 3:
        while index < len(data) and data[index] in b" \t\r\n":
            index += 1
        if index < len(data) and data[index] == ord("#"):
            end = data.find(b"\n", index)
            index = len(data) if end < 0 else end + 1
            continue
        if index >= len(data):
            raise ValueError(f"{path} has an incomplete PGM header")
        end = index
        while end < len(data) and data[end] not in b" \t\r\n":
            end += 1
        tokens.append(data[index:end])
        index = end

    width, height, maximum = (int(token) for token in tokens)
    if width <= 0 or height <= 0 or maximum != 255:
        raise ValueError(f"{path} has unsupported PGM dimensions or maximum value")
    if index >= len(data) or data[index] not in b" \t\r\n":
        raise ValueError(f"{path} has no separator before the PGM payload")
    # Consume the header delimiter only. Pixel bytes may themselves be
    # whitespace, so skipping an arbitrary run here would corrupt the image.
    if data[index:index + 2] == b"\r\n":
        index += 2
    else:
        index += 1
    pixels = data[index:]
    expected = width * height
    if len(pixels) != expected:
        raise ValueError(f"{path} payload is {len(pixels)} bytes, expected {expected}")
    return width, height, pixels


def _yaml_number(text: str, key: str, path: Path) -> float:
    pattern = rf"^{re.escape(key)}:\s*([-+0-9.eE]+)\s*$"
    match = re.search(pattern, text, re.MULTILINE)
    if match is None:
        raise ValueError(f"{path} is missing YAML field: {key}")
    return float(match.group(1))


def read_map_yaml(path: str | Path) -> tuple[float, float, float, int, float, float]:
    """Read resolution, ENU origin and occupancy thresholds from map YAML."""

    metadata_path = Path(path)
    text = metadata_path.read_text(encoding="utf-8")
    resolution = _yaml_number(text, "resolution", metadata_path)
    origin_match = re.search(
        r"^origin:\s*\[([^]]+)\]\s*$", text, re.MULTILINE
    )
    if origin_match is None:
        raise ValueError(f"{metadata_path} is missing YAML field: origin")
    origin_values = [float(value.strip()) for value in origin_match.group(1).split(",")]
    if len(origin_values) != 3:
        raise ValueError(f"{metadata_path} origin must contain three values")

    negate_match = re.search(r"^negate:\s*(-?\d+)\s*$", text, re.MULTILINE)
    negate = int(negate_match.group(1)) if negate_match else 0
    occupied_thresh = _yaml_number(text, "occupied_thresh", metadata_path)
    free_thresh = _yaml_number(text, "free_thresh", metadata_path)
    return (
        resolution,
        origin_values[0],
        origin_values[1],
        negate,
        occupied_thresh,
        free_thresh,
    )


def normalize_cells(
    pixels: bytes,
    *,
    negate: int,
    occupied_thresh: float,
    free_thresh: float,
) -> bytes:
    """Convert standard map probabilities to planner values."""

    if not 0.0 <= free_thresh < occupied_thresh <= 1.0:
        raise ValueError("Occupancy thresholds must satisfy 0 <= free < occupied <= 1")

    cells = bytearray(len(pixels))
    for index, pixel in enumerate(pixels):
        probability = pixel / 255.0 if negate else (255 - pixel) / 255.0
        if probability > occupied_thresh:
            cells[index] = 100
        elif probability < free_thresh:
            cells[index] = 0
        else:
            cells[index] = 255
    return bytes(cells)


def load_occupancy_map(
    image_path: str | Path,
    metadata_path: str | Path | None = None,
) -> OccupancyMap:
    """Load a standard PGM/YAML occupancy map for the global planner."""

    image_path = Path(image_path)
    metadata_path = Path(metadata_path or image_path.with_suffix(".yaml"))
    width, height, pixels = read_pgm(image_path)
    (
        resolution,
        origin_east,
        origin_north,
        negate,
        occupied_thresh,
        free_thresh,
    ) = read_map_yaml(metadata_path)
    return OccupancyMap(
        width=width,
        height=height,
        resolution_m=resolution,
        origin_east_m=origin_east,
        origin_north_m=origin_north,
        cells=normalize_cells(
            pixels,
            negate=negate,
            occupied_thresh=occupied_thresh,
            free_thresh=free_thresh,
        ),
    )
