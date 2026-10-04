# Charanya — Perception 3: artefact localisation and display

## What this delivers

Assignment pages 10–11 require estimated artefact map positions, combining repeated observations, and RViz markers. This code estimates a visible object surface location from a detection and live depth, transforms it to `map`, averages nearby same-class sightings, and shows a labelled sphere after three observations. It keeps separate tracks for objects of the same class when they are sufficiently far apart.

Based on the attached September 14 ZIP and the earlier `Pasted text (2).txt` Perception 1 integration. The ZIP has the starter stop-sign detector, simulator models and textures; these textures are not a captured training dataset. No trained multi-class Perception 2 model or captured dataset was available. The supplied working detector connection therefore uses `stop_sign` only. Do not describe this as a tested all-artefact system.

Perception 1 captures photos → Perception 2 detects class and bounding box → Perception 3 estimates map position. Perception 3 does not train on the photo folders. RGB photos alone do not contain metric depth or timestamped robot poses.

## Install in your Ubuntu ROS workspace

1. Extract this ZIP. Find your existing `cave_explorer` package: the correct folder contains `setup.py`, `package.xml`, `launch/` and another `cave_explorer/` directory.
2. From the extracted `perception3` folder, run (adjust the package path):

```bash
python3 install_perception3.py ~/ros_ws/src/cave_explorer
```

The installer backs up `cave_explorer.py`, preserves existing Perception 1 methods and planning, adds the two helper modules, replaces the localisation placeholders, fixes the starter's swapped bounding-box width/height, and preserves the annotated image timestamp. It supports the supplied starter and the earlier P1 version. It refuses unrecognised versions rather than silently editing a different detector.

If using the earlier P1 version, keep your existing `perception1_dataset.py` alongside it. That separate helper was not in the uploaded ZIP and is not recreated here.

3. Ensure dependencies are available:

```bash
sudo apt install python3-numpy ros-humble-cv-bridge ros-humble-tf2-ros ros-humble-visualization-msgs
cd ~/ros_ws
colcon build --symlink-install --packages-select cave_explorer
source install/setup.bash
```

The provided package already imports the ROS message types and TF. If maintaining dependency declarations, ensure `package.xml` lists runtime dependencies for `python3-numpy`, `cv_bridge`, `sensor_msgs`, `geometry_msgs`, `visualization_msgs`, `rclpy`, and `tf2_ros`.

4. Run the original three launches, one per sourced terminal:

```bash
ros2 launch cave_explorer cave_explorer_startup.launch.py
ros2 launch cave_explorer cave_explorer_navigation.launch.py
ros2 launch cave_explorer cave_explorer_autonomy.launch.py
```

Keep simulation time enabled (the existing launch defaults to True). Do not run a second old autonomy node at the same time.

5. In RViz choose Fixed Frame `map`; add/enable a **MarkerArray** display with topic `/marker_array_artifacts`. The original display may already exist. Keep `/detections_image` visible. A marker needs three valid RGB/depth/TF observations; the marker label includes the class, ID, coordinates and observation count.

## Connecting your teammate's Perception 2 detector

The installed stop-sign adapter is inside `image_callback`:

```python
self.localise_artifact(
    image_msg,
    [('stop_sign', tuple(box)) for box in detections]
)
```

Replace that call with actual labelled detections when Perception 2 is ready:

```python
# Illustrative interface only: use your detector's real outputs.
# Bounding boxes must be in ORIGINAL 720 x 480 image pixels.
p3_detections = [
    (class_name, (float(x), float(y), float(width), float(height)))
    for class_name, x, y, width, height in your_detector_results
]
self.localise_artifact(image_msg, p3_detections)
```

`your_detector_results` is an interface example, not an existing variable in your uploaded code. If a detector returns `(x1,y1,x2,y2)`, convert to `(x1,y1,x2-x1,y2-y1)`. Undo resizing/letterboxing before submitting boxes. Pass all detections in one call per image; perform detector confidence filtering and nonmaximum suppression upstream. Class names consistent with P1 are `green_alien`, `green_crystals`, `white_sphere`, `ice_formation`, `blue_mushrooms`, `stop_sign`; `negative` is ignored. Other names work with a default marker colour.

For manual integration in a newer teammate version: import `Perception3`, instantiate `self.perception3 = Perception3(self)` after `cv_bridge_`, `tf_buffer`, `marker_pub_`, and `artifact_locations_` exist; call `self.perception3.submit(image_msg, labelled_boxes)` once per image. Remove old robot-position marker updates. Copy both helper files beside `cave_explorer.py`. This helper assumes the existing single-threaded executor.

## How your code works

1. Subscribe to existing `/camera/depth/image`, using sensor-data QoS.
2. Match RGB observations to a depth timestamp within 0.08 s. Briefly queue detections for delayed depth/TF. Never substitute a robot position when depth is missing.
3. Use a small central bounding-box patch, discard invalid/zero/out-of-range depths, and take median depth. Depths between 0.4 and 10 m are used conservatively.
4. The provided sensor has width 720, height 480 and horizontal FOV 2.0944 radians. Set `f = 720/(2*tan(2.0944/2))`. For image coordinate `(u,v)` and axial depth `d`, the camera-link point is `(d, -(u-360)*d/f, -(v-240)*d/f)`.
5. Apply the full quaternion and translation from TF `map <- camera_link` at the RGB timestamp. The simulator sensor is attached directly to `camera_link`. Its header says `camera_link`; the separate offset URDF optical links are not used. This avoids applying an optical-axis convention directly to a forward-axis frame.
6. Associate with the nearest same-class track within 0.75 m, with one update per track per image. Update the running average. Require three observations before publication; discard unconfirmed tracks after five seconds when processing subsequent observations.
7. Publish coloured spheres and text with stable track IDs. Republish every second so RViz opened later receives the results. `artifact_locations_` contains confirmed `geometry_msgs/Point` positions for existing planning code. Planning must choose an accessible stand-off goal, not drive into these surface points.

The code uses 32FC1 depth as metres and 16UC1 as millimetres, consistent with ROS REP 118: https://reps.openrobotics.org/rep-0118/ . Timestamped TF uses the ROS 2 buffer API: https://docs.ros.org/en/humble/p/tf2_ros_py/tf2_ros.buffer.html .

## What to verify in Gazebo

This environment does not have ROS/Gazebo, so runtime integration and localisation accuracy remain unverified. The included unit tests cover geometric projection, transform rotation/translation, invalid depths, track fusion, class separation and provisional-track expiry. Synthetic tests do not establish actual simulator accuracy.

- Check `ros2 topic echo /camera/depth/image --once --field header` reports `camera_link`; check RGB too. Do not simply bypass a frame warning if your camera configuration differs.
- Check `ros2 run tf2_ros tf2_echo map camera_link` while navigation is running.
- Observe a stop sign from several poses. Its marker should remain near its visible surface instead of following the robot. Record map-coordinate variation and a screenshot.
- Test two same-class artefacts and adjust `p3_merge_radius` if needed: `ros2 param set /cave_explorer_node p3_merge_radius 0.5`. `p3_sync_tolerance` and `p3_hfov` are read at processing time.
- When the camera sees no artefact, no new marker should be created. Previously confirmed markers remain.
- Cover invalid-depth regions and confirm the observation is skipped. Restart autonomy after restarting/resetting the simulation or map.

Limitations: the central patch can contain background or an occluder; a wrong detector can still produce a wrong location. It estimates a visible surface, not an object's hidden centre. Close same-class objects can merge and large pose errors can split one object. SLAM loop-closure corrections do not retroactively move old observations; restart/reobserve for a fresh map. It assumes the attached, co-located Gazebo RGB-D sensor, axial depth, square pixels and unchanged camera configuration. CameraInfo calibration/registration is needed for other cameras. No world-file artefact coordinates are used.

## Tests and report

```bash
python3 -m unittest discover -s . -p 'test_geometry.py' -v
```

For the report, describe the projection, timestamped TF, averaging/association and limits above. Include screenshots and your measured results after running the simulation; do not claim an accuracy figure until measured. The assignment explicitly requires a brief generative-AI disclosure. Explain that ChatGPT assisted implementation and that you reviewed, tested and understood the code, once you have done so.
