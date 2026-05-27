
```bash
ros2 run barrel_leak_cpp detect_barrel_cpp
```

## Run With The Config File

Build + run: 

```bash
ros2 run barrel_leak_cpp detect_barrel_cpp --ros-args \
  --params-file src/barrel_leak_cpp/config/barrel_leak_cpp.yaml
```
Example with topic overrides:

```bash
ros2 run barrel_leak_cpp detect_barrel_cpp --ros-args \
  --params-file src/barrel_leak_cpp/config/barrel_leak_cpp.yaml \
  -p image_topic:=/oakd/rgb/preview/image_raw \
  -p point_cloud_topic:=/oakd/rgb/preview/depth/points \
  -p target_frame:=map
```

## Debug Mode

The node always publishes debug image topics when the `enable_debug_*`
parameters are true. To also open local OpenCV windows, set:

```bash
ros2 run barrel_leak_cpp detect_barrel_cpp --ros-args \
  --params-file src/barrel_leak_cpp/config/barrel_leak_cpp.yaml \
  -p show_debug_window:=true
```

Useful debug parameters:

- `enable_debug_overlay`: publish/open the main accepted-candidate overlay.
- `enable_debug_mask`: publish/open the combined HSV color mask.
- `enable_debug_rejections`: publish/open rejected candidates with reasons.
- `enable_debug_depth_alignment`: publish/open RGB/depth alignment hints.
- `enable_debug_depth_validity`: publish/open finite-depth coverage.
- `enable_debug_leak_overlay`: publish/open leak search and leak candidates.
- `show_debug_window`: open local OpenCV windows in addition to publishing topics.

Debug windows:

- `barrel`: camera image with barrel outlines, track ids, accepted state, orientation arrows, and fit metrics.
- `barrel_mask`: combined color mask for all configured barrel colors.
- `barrel_rejections`: candidates that failed gates, labelled with reasons like `area`, `depth`, `ransac`, `height`, or `orientation`.
- `barrel_depth_alignment`: for masks with no depth, shows the nearby pixel shift that would find the most valid depth samples.
- `barrel_depth_validity`: depth coverage view. Green means valid depth, yellow means mask plus valid depth, red means mask with no usable depth.
- `barrel_leak`: leak debug overlay. It shows the barrel search area, rejected leak blobs, accepted leak blobs, and compact candidate metrics.

You can inspect the same images without local windows using:

```bash
ros2 run rqt_image_view rqt_image_view
```

Then select one of:

- `/barrel/debug_overlay`
- `/barrel/debug_mask`
- `/barrel/debug_rejections`
- `/barrel/debug_depth_alignment`
- `/barrel/debug_depth_validity`
- `/barrel/debug_leak_overlay`



## Barrel Detection Flow

The detector synchronizes the RGB image and organized point cloud. It builds HSV
masks for red, green, blue, yellow, purple, orange, brown, and black, extracts
contours, then samples 3D points inside each contour. Candidates must pass image
size/fill gates, 3D extent gates, PCL Euclidean clustering, and PCL cylinder
RANSAC.

After a cylinder is fitted, the candidate is transformed into `target_frame`.
The cylinder axis is classified as horizontal or vertical using the configured
axis thresholds, and horizontal barrels get a normalized `normal_x`/`normal_y`.
Tracks are accepted after `accept_threshold` supporting frames. Accepted tracks
publish `BarrelDetect` with position, color, horizontal state, orientation
normal, and leak status. A track republishes only when it first appears, moves
far enough, changes orientation, changes color, or changes leak state.
Detections are associated to an existing track only when the detected color
matches the track color and the XY map distance is within `dedup_distance_m`.
Never-published tentative tracks are removed after `track_timeout_frames` missed
camera callbacks. Tracks that have already published on `/barrel_detect` are
kept so later observations can reconnect to the same id.

## Leak Detection Flow

For each horizontal detected barrel, the leak overlay searches a padded image
ROI around the barrel bbox for HSV color-mask blobs. RANSAC cylinder inlier
pixels are no longer removed from the leak mask, so leaks touching the barrel
can still be detected. A leak blob must pass 2D shape gates for area, fill,
circularity, and axis ratio, then have enough organized point-cloud samples
whose source-Z values fall inside `leak_source_z_min_m` to
`leak_source_z_max_m` at the configured inlier ratio. Map-frame height,
distance from the barrel, and thickness remain secondary gates. Accepted leak
blobs update the track's `BarrelDetect.leaking` boolean after the configured
confirmation threshold, and clear it after the clear threshold.
