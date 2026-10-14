# Planning 3 + Advanced 1 — Charanya's implementation

## What you received

This is an additive code bundle for **the exact attached `49274-main (1).zip`**. Its starter `cave_explorer.py` is byte-for-byte identical to the earlier upload. It contains only the starter stop-sign detector and random/predefined navigation; no completed teammate Planning 1, Planning 2, multi-class detector, or captured dataset is present.

Your requested parts are `planning3.py` (behaviour switching) and `advanced1.py` (robust perception). The other files connect them to ROS, provide basic exploration/approach support, and reuse the earlier Perception 3 depth-localisation implementation. The new launch replaces the original autonomy launch; simulator/navigation launches stay the same. The installer leaves your original `cave_explorer.py` untouched.

This implements the requested behaviours and supplies a reproducible evaluation workflow. It is **not a claim of completed Gazebo demonstrations or proven perception improvement**. No ROS/Gazebo runtime or your captured dataset was available here. The supplied detector supports **stop signs only**. Planning 2 asks for two or three reliable classes; your team must supply its Perception 2 detector before claiming that broader coverage.

## 1. Install

Extract the original project into your ROS workspace if it is not already there. The package folder must contain `setup.py`, `package.xml`, `launch/`, `config/`, and a `cave_explorer/` subfolder. Avoid having two copies of the same package inside the workspace.

Extract this code ZIP. Open a terminal in the extracted `planning3_advanced1` folder and run:

```bash
source /opt/ros/humble/setup.bash
python3 install.py ~/ros_ws/src/cave_explorer
```

Replace `~/ros_ws/src/cave_explorer` with your actual package folder. In the attached ZIP it is nested under `49274-main/cave_explorer-3/cave_explorer`.

The installer adds modules, a launch file, a `cave_mission` entry point, and runtime dependencies. It backs up `setup.py` and `package.xml` with `.before_p3_a1` suffixes and refuses conflicting module files. Use a fresh copy of the attached package if another version of the earlier Perception 3 helpers conflicts. Do not overwrite teammate changes blindly.

Install the Python/image dependencies in the Ubuntu ROS environment (retain the assignment's Gazebo/Nav2/SLAM installations):

```bash
sudo apt install python3-numpy python3-opencv ros-humble-cv-bridge ros-humble-tf2-ros ros-humble-visualization-msgs
cd ~/ros_ws
colcon build --symlink-install --packages-select cave_explorer
source install/setup.bash
```

Use OpenCV 4.x, as provided by Ubuntu 22.04 `python3-opencv`; this starter Haar-cascade detector is not compatible with OpenCV 5 builds that omit CascadeClassifier. No pip or PyTorch installation is needed on your ROS computer.

## 2. Run Planning 3

Use three separate terminals. In each, first run:

```bash
source /opt/ros/humble/setup.bash
source ~/ros_ws/install/setup.bash
```

Terminal 1:

```bash
ros2 launch cave_explorer cave_explorer_startup.launch.py
```

Terminal 2:

```bash
ros2 launch cave_explorer cave_explorer_navigation.launch.py
```

Terminal 3 (the NEW autonomy launch):

```bash
ros2 launch cave_explorer planning3_advanced1.launch.py
```

**Do not also run `cave_explorer_autonomy.launch.py` or your teammate's independent goal-sending node.** There must be one owner of Nav2 navigation goals.

The clean baseline is selected initially (`degradation=none`, `severity=0`, `use_improved=false`). The robot selects free reachable exploration goals near unknown space. On three consistent stop-sign localisation observations, it switches to inspection. It uses a 2 m standoff viewpoint, checks map clearance and line of sight, and faces the artefact at the goal. Successful arrival must be followed by a fresh detection at the viewpoint before the artefact is marked visited.

During an approach, no sighting for 8 seconds or a 45-second approach timeout triggers cancellation and one retry from an alternative viewpoint. A second failure marks the artefact abandoned for the current run and resumes exploration. Cancellation is asynchronous: the next navigation goal is not sent until the previous action has a terminal result. If cancellation never resolves, the state becomes HALTED instead of issuing competing goals.

State flow:

`EXPLORE → CANCELLING → APPROACH_START → APPROACH → INSPECT → RESUME → EXPLORE`

Failure path: `APPROACH → CANCELLING → RETRY → APPROACH`; another failure leads to `RESUME`. If there was no active goal, the cancellation step is skipped.

Inspect status in another terminal:

```bash
ros2 topic echo /planning3/status
```

In RViz:

| Display type | Topic | Purpose |
|---|---|---|
| MarkerArray | `/marker_array_artifacts` | Estimated artefact positions and classes |
| MarkerArray | `/planning3/markers` | UNVISITED, TARGET, VISITED, ABANDONED labels |
| Pose | `/goal_pose` | Current approach or exploration goal |
| Image | `/detections_image` | Selected detector output |

Use Fixed Frame `map`. Existing RViz map/robot displays remain useful.

## 3. Run Advanced 1 live

Stop only Terminal 3 with Ctrl+C before changing modes. Keep simulator and navigation running. For low light at severity 2, with the candidate improved pipeline controlling detections:

```bash
ros2 launch cave_explorer planning3_advanced1.launch.py degradation:=low_light severity:=2 use_improved:=true
```

For noise:

```bash
ros2 launch cave_explorer planning3_advanced1.launch.py degradation:=noise severity:=2 use_improved:=true
```

For visual/perception testing without autonomous goal generation:

```bash
ros2 launch cave_explorer planning3_advanced1.launch.py planning_enabled:=false degradation:=low_light severity:=2
```

You may use teleoperation or RViz goals during dataset collection/testing. The final autonomous demonstration must not depend on manual navigation.

Live changes to image processing are supported:

```bash
ros2 param set /cave_mission degradation noise
ros2 param set /cave_mission severity 3
ros2 param set /cave_mission use_improved true
```

Set `planning_enabled`, `chosen_types`, standoff and planning timeouts at startup; those are read once. Restart autonomy for an independent mission comparison, since visited/abandoned records and localisation tracks persist during a run. Restart the simulator too if you need a matched initial pose/map for end-to-end comparisons.

Add these Image displays in RViz:

| Topic | Image |
|---|---|
| `/robust/original` | Original colour image |
| `/robust/degraded` | Corrupted colour image |
| `/robust/restored` | Candidate restored image |
| `/robust/clean_detections` | Clean-reference detections |
| `/robust/baseline_detections` | Detector on corrupted image |
| `/robust/improved_detections` | Same detector after restoration |

Only the selected baseline/improved detections feed localisation and planning. Clean detections are displayed for reference, not secretly used to guide navigation when degradation is enabled.

### Degradation settings

| Type | Severity 1 | Severity 2 | Severity 3 |
|---|---:|---:|---:|
| Low light | brightness × 0.65 | × 0.40 | × 0.20 |
| Gaussian image noise | σ = 10 | σ = 25 | σ = 45 |

Noise sigma uses 0–255 pixel units. Severity 0 or type `none` leaves the source image unchanged. Arrays retain their original dimensions, so boxes and depth pixels remain aligned.

The proposed improvement is bilateral denoising, brightness correction with an image-derived gamma, and CLAHE on luminance. The restoration uses only the corrupted image; it is not given the clean reference or the corruption strength. These methods can also suppress useful features or amplify noise, so improvement is a hypothesis to test, not an assumed result. Keep and report negative results too. If results do not improve, tune the pipeline/model using validation images, then evaluate once on held-out test images.

## 4. Collect evidence and measure accuracy

Results are saved under `~/cave_results/<run timestamp>/`:

- `mission_events.jsonl`: state transitions, exploration/approach goals, visited and abandoned events.
- `live_counts.csv`: clean/baseline/improved detection counts and full live-pipeline latency. **Counts are not precision, recall or accuracy.**
- `captures/`: clean frames saved only when requested.

To save one clean image:

```bash
ros2 topic pub --once /robust/capture_save std_msgs/msg/Empty '{}'
```

Collect varied distances, angles, backgrounds, partially occluded targets and genuine negative images. Perception 1 folder labels do not provide bounding-box annotations. You can use those existing photos, but must label every target box for object-detection evaluation. Avoid near-identical video frames in different train/validation/test subsets; split by collection session or viewpoint. Include all stop signs visible in an image, even if a folder was named for another object.

From the extracted `planning3_advanced1` folder, label your evaluation images:

```bash
python3 tools/label_images.py /absolute/path/to/evaluation_images --output evaluation_labels.json
```

The desktop OpenCV window lets you draw all stop-sign boxes. ENTER/SPACE accepts a box; ESC ends that image. For a genuine negative image, press ESC without drawing any boxes. The tool checkpoints the manifest after every image. Finish labelling all intended images before evaluation.

Run the paired evaluator:

```bash
python3 tools/evaluate_advanced1.py \
  --manifest evaluation_labels.json \
  --model ~/ros_ws/src/cave_explorer/config/stop_data.xml \
  --output evaluation_results
```

Adjust the model path. Use a new output folder for each run. The evaluator applies both corruption types at all three levels to the **same labelled test images**, compares baseline versus restoration, and also includes an uncorrupted control. It uses one-to-one same-class box matches at IoU ≥ 0.5, so duplicate or wrong detections count as false positives and missed boxes count as false negatives.

Outputs:

| File | Meaning |
|---|---|
| `summary.csv` | TP, FP, FN, precision, recall, F1 and mean inference latency per condition |
| `per_image.csv` | Results for each image and condition |
| `comparison.json` | Improved-minus-baseline F1 and latency differences |
| `examples/` | Annotated examples: green predicted boxes; magenta human ground truth |
| `provenance.json` | Seed, model/image hashes, annotations, OpenCV version and IoU threshold |

Blank metrics mean an undefined denominator, not perfect performance. This is detection F1 at IoU 0.5, **not mAP**. Latency includes restoration for the improved pipeline; it excludes corruption generation and disk IO. The live logger includes all three detector passes, restoration and publishing, so its time should not be compared directly with per-pipeline evaluator timing. This evaluator measures RGB detection robustness; it does not measure map-coordinate localisation error or mission completion performance. Those need separate Gazebo trials.

## 5. Connect your teammates' modules

The default stop-sign detector is intentionally explicit. To use your Perception 2 model:

1. Implement `detect(bgr) -> list[Detection]` in `advanced1.py`, and expose its supported `labels`.
2. Each `Detection` has a class string and `(x,y,width,height)` in the original 720×480 image. Undo detector resizing/letterboxing and apply confidence filtering/nonmaximum suppression upstream.
3. Use the same detector adapter in `mission_node.py` and `tools/evaluate_advanced1.py` so evaluation matches the live system.
4. Set `chosen_types` to two or three reliably detected classes before starting the node. P1 names are `green_alien`, `green_crystals`, `white_sphere`, `ice_formation`, `blue_mushrooms`, `stop_sign`; `negative` is not an artefact.

To reuse your teammate's Planning 1/2 logic, replace `GridPlanner.explore(robot)` and `GridPlanner.approach(target,robot,attempt,standoff)` with functions returning `(x,y,yaw)` goals or `None`. They must not independently send Nav2 goals: `NavSession` owns those to preserve cancellation and switching order.

The helper's map screening uses known free cells, a 0.75 m clearance threshold (the provided Nav2 robot radius is 0.5 m), connected-component reachability and a simple line-of-sight screen. Nav2 still determines actual path feasibility. Exploration is a basic frontier-proximity strategy with revisitation avoidance, not an optimised complete coverage algorithm. It can wait with no suitable goals or miss disconnected regions; improve it through Planning 1 work. Approach goals face the object at arrival; the current Nav2 controller may turn away during a detour. The lost-target timeout handles this but continuous visual servoing is not implemented.

Perception 3 assumes the supplied Gazebo camera: aligned 720×480 RGB/depth, FOV 2.0944 rad, frame `camera_link`, depth in metres (32FC1) or millimetres (16UC1). It estimates visible surface positions and averages same-class tracks. Poor boxes/depth/SLAM can merge/split targets; visited/abandoned spatial deduplication uses a 1 m radius and is only kept for the current run. SLAM loop closures do not retroactively correct stored observations. Restart the mission node after resetting simulation/map.

## 6. Demo and report checklist

For Planning 3, record:

- Autonomous exploration followed by interruption on a new target.
- Approach and fresh detection at the inspection viewpoint.
- VISITED marker, then resumed exploration.
- Passing the same artefact again without another inspection.
- A lost/failed approach, one retry, abandonment and resumed exploration.

Use `mission_events.jsonl` for counts of unique inspected targets, retries and abandoned attempts. Do not label a navigation abort as a successful visit. To stage a lost-target test during development, severe degradation can suppress detections, but verify the actual output; it is not guaranteed to make every target disappear. Include an ordinary autonomous run for the assessed demonstration.

For Advanced 1, record original, degraded and detection outputs in RViz. Show both degradation types at several levels. Include your real held-out precision/recall/F1 table and before/after examples. Discuss false positives on negatives and runtime cost. If performance worsens, report that honestly and make another validated improvement rather than inventing gains.

Assignment pages 12–13 ask for the method, difficulties and evidence, not merely a code listing. The report must also disclose generative-AI assistance and accurately describe your contribution. Read and understand these files before the live questions.

## 7. Tests and limitations of verification

Run the non-ROS checks from this extracted folder:

```bash
python3 -m unittest discover -s tests -v
```

Tests exercise state transitions, retry/abandonment, fresh confirmation, duplicate avoidance, cancellation races with API doubles, image corruption, matching metrics, camera geometry and grid screening. Action tests use doubles and do not establish ROS wire compatibility. Installer and Python syntax checks were also performed. ROS 2 Humble/Gazebo navigation, real camera headers, end-to-end timings and detection accuracy must still be verified on your Ubuntu system.

Useful diagnostics:

```bash
ros2 topic echo /camera/depth/image --once --field header
ros2 run tf2_ros tf2_echo map camera_link
ros2 action list
ros2 topic echo /planning3/status
```

If no markers appear, check detections, valid depths, `camera_link` headers, TF at image timestamps, and the requirement for three observations. If no movement occurs, check Nav2 availability, map/TF, known free space, and HALTED status. Do not bypass the single-action cancellation guard. The node stops sending new goals on a stuck cancellation; a terminal result is required before it can safely resume. Ctrl+C attempts cancellation briefly; stop simulation/navigation explicitly if a faulty action server does not respond.

Manual installation, if `setup.py` differs: copy the bundle's `cave_explorer/*.py` into the existing Python package and the launch file into `launch/`, then add `'cave_mission = cave_explorer.mission_node:main'` to `console_scripts`, add the dependencies listed in `install.py`, and rebuild. Back up existing files and merge teammate modifications first.

## Technical references

- Assignment: Exploring a Martian Cave, 49274 (2026), Planning 3 p.12 and Advanced 1 p.13.
- ROS 2 action clients: https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html
- OpenCV filtering: https://docs.opencv.org/4.x/d4/d13/tutorial_py_filtering.html
- OpenCV CLAHE: https://docs.opencv.org/4.x/d5/daf/tutorial_py_histogram_equalization.html
- Depth-image units: https://reps.openrobotics.org/rep-0118/
