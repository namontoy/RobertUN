from dataclasses import dataclass
from pathlib import Path

import cv2


VALID_COLORS = ('red', 'white', 'black')


@dataclass
class DetectionResult:
    success: bool
    count: int
    summary: str
    output_image_path: str


def count_cars_by_color(
    image_path,
    target_color,
    min_area=450.0,
    save_output_image=False,
    output_dir='',
):
    target_color = str(target_color).lower()
    if target_color not in VALID_COLORS:
        valid = ', '.join(VALID_COLORS)
        return DetectionResult(
            False,
            0,
            f"Invalid target_color='{target_color}'. Use one   bash scripts/image_tools/request_color_count.shof: {valid}",
            '',
        )

    if not image_path:
        return DetectionResult(False, 0, 'image_path is required', '')

    path = Path(str(image_path)).expanduser()
    if not path.exists():
        return DetectionResult(False, 0, f'Image path does not exist: {path}', '')

    image = cv2.imread(str(path))
    if image is None:
        return DetectionResult(False, 0, f'OpenCV could not read image: {path}', '')

    mask = _build_color_mask(image, target_color)
    mask = _clean_mask(mask)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    candidates = _filter_candidates(contours, image.shape, float(min_area))
    output_image_path = ''

    if save_output_image:
        output_image_path = _save_annotated_image(
            image,
            path,
            target_color,
            candidates,
            output_dir,
        )

    summary = f"Detected {len(candidates)} likely {target_color} cars in {path.name}"
    return DetectionResult(True, len(candidates), summary, output_image_path)


def _build_color_mask(image, target_color):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    if target_color == 'red':
        lower_red_a = (0, 70, 50)
        upper_red_a = (10, 255, 255)
        lower_red_b = (170, 70, 50)
        upper_red_b = (180, 255, 255)
        mask_a = cv2.inRange(hsv, lower_red_a, upper_red_a)
        mask_b = cv2.inRange(hsv, lower_red_b, upper_red_b)
        return cv2.bitwise_or(mask_a, mask_b)

    if target_color == 'white':
        lower_white = (0, 0, 150)
        upper_white = (180, 80, 255)
        return cv2.inRange(hsv, lower_white, upper_white)

    lower_black = (0, 0, 0)
    upper_black = (180, 255, 85)
    return cv2.inRange(hsv, lower_black, upper_black)


def _clean_mask(mask):
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def _filter_candidates(contours, image_shape, min_area):
    image_area = image_shape[0] * image_shape[1]
    candidates = []

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < min_area:
            continue

        x, y, width, height = cv2.boundingRect(contour)
        if width == 0 or height == 0:
            continue

        aspect_ratio = width / float(height)
        area_ratio = area / float(image_area)

        if 0.45 <= aspect_ratio <= 4.5 and area_ratio <= 0.08:
            candidates.append((x, y, width, height, area))

    return candidates


def _save_annotated_image(image, image_path, target_color, candidates, output_dir):
    output_root = Path(output_dir).expanduser() if output_dir else image_path.parent
    output_root.mkdir(parents=True, exist_ok=True)

    output_path = output_root / f'{image_path.stem}_{target_color}_detected.png'
    annotated = image.copy()

    box_color = _box_color(target_color)
    for x, y, width, height, _area in candidates:
        cv2.rectangle(annotated, (x, y), (x + width, y + height), box_color, 2)

    cv2.imwrite(str(output_path), annotated)
    return str(output_path)


def _box_color(target_color):
    if target_color == 'red':
        return (0, 0, 255)
    if target_color == 'white':
        return (255, 255, 255)
    return (0, 255, 0)
