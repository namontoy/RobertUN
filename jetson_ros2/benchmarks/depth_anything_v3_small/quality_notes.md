# Depth Anything V3 Small Visual Review

## Evidence status

These are qualitative observations from the eight recorded benchmark videos.
They complement the timing CSV but are not ground-truth depth measurements.
No ZED camera was used.

## `busyCity.mp4`

- Useful depth: uncertain
- Flicker: low
- Edge quality: medium
- Relative-depth consistency: medium
- Notes: Cars far from the camera are difficult to separate reliably.

## `car_video.mp4`

- Useful depth: yes
- Flicker: low
- Edge quality: high
- Relative-depth consistency: high
- Notes: A few frames become purple or black during fast motion. Smooth parts
  of the video produce a stable and useful depth visualization.

## `forest.mp4`

- Useful depth: yes
- Flicker: low
- Edge quality: medium
- Relative-depth consistency: high
- Notes: Grass and bushes have weak boundaries where color and relative depth
  are similar.

## `hamilton.mp4`

- Useful depth: uncertain
- Flicker: high
- Edge quality: low
- Relative-depth consistency: medium
- Notes: This was the weakest video in the set, although some frames remain
  useful. The distant car is not consistently separated.

## `humanoidrobot.mp4`

- Useful depth: yes
- Flicker: low
- Edge quality: high
- Relative-depth consistency: high
- Notes: The depth visualization is stable across most of the short clip.

## `lion-buffalo.mp4`

- Useful depth: yes
- Flicker: low
- Edge quality: high
- Relative-depth consistency: high
- Notes: Individual animals are visually separated. Similar colors and small
  depth differences still make some regions ambiguous. This review concerns
  depth quality, not YOLO class accuracy.

## `man_walking.mp4`

- Useful depth: yes
- Flicker: medium
- Edge quality: high
- Relative-depth consistency: high
- Notes: Distant people flicker, so their relative depth is less reliable.

## `peoplewalking.mp4`

- Useful depth: yes
- Flicker: low
- Edge quality: high
- Relative-depth consistency: high
- Notes: Most people remain clearly separated in the depth visualization.
