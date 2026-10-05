# detection-dock modular service

*detection-dock* is a Viam modular service that provides docking capabilities using a [vision service detector](https://docs.viam.com/services/vision/detection/)

The model this module makes available is viam-labs:dock:detection-dock

Docking follows the same stages as the [Nav2 docking server](https://docs.nav2.org/rolling/tutorials/general_tutorials/using_docking/): find the dock, then run a vision-control loop that continuously refines the target while driving toward it. The approach uses Nav2's graceful control law (bearing and range estimated from the detection). There is no map or staging navigation — if the dock is not in view, the base spins until the detector sees it.

1. Watch for `detection_class` while holding still for `search_settle` seconds, so a detection already in frame is not spun away. If it is still missing, spin at `search_angular_velocity` until the detector sees it, or until the base has turned `search_spin_deg` (default two full rotations).
2. Enter the vision-control loop. Each cycle, estimate bearing from where the detection sits in the image and range from how large it is, filter that pose, and command a smooth velocity toward it. The approach steers on that image bearing. Surface tilt from depth is not used until the detection is wide, because a far reading sits near -27 degrees no matter where the dock is in the image and turning to chase it swings the dock out of frame. Inside the slowdown distance the turn is limited to `close_angular_max`. Within about 5 inches the base keeps driving in while the detection is still growing. A tilted surface is corrected only after the detection is within about 0.06 of `close_percent` and has not grown for a second: then it backs up at most 10 cm while turning for that whole reverse. The turn follows the surface yaw until the dock leaves the center of the image. Spinning in place would walk the dock off the side of the camera, and driving forward at an angle is stopped by obstacle avoidance. The reverse is kept short so the base does not hit whatever is behind it. Once it is square and centered, it creeps forward if the detection is still small. Surface tilt is used only when the detection is wide and the tilt is within 20 degrees. That tilt is the median of recent depth samples, and a frame that jumps by more than about 20 degrees is ignored. Once the detection is already at least `close_percent` wide, the base holds position instead of backing up. A turn starts only after the base has stopped and the filtered error has held one direction for about half a second.
3. Leave the loop once the detection is centered within `center_tolerance` and at least `close_percent` of the image wide, and that alignment has held for about half a second while the base is stopped. With depth, the surface also has to be within `surface_yaw_tolerance_deg` of straight on. A centered detection can still be tilted, which makes the target look smaller than it does when the robot is square.
4. If `power_sensor` is set, wait up to `wait_charge_timeout` for the voltage to rise by `charge_voltage_delta`. If it does not, back up and retry, up to `max_retries`. That retry reverse is an arc that takes out the yaw left over from the failed try, so the next approach starts squarer. If `power_sensor` is omitted, reaching the target is success.

This has been tested with [feature match detection](https://github.com/viam-labs/feature-match-detector) configured as a vision detector, but other detector types should work, as well.
For docking with the feature match detector, we used the following image, but you can use others:
![dock image](./charge.jpg)

## Prerequisites

For linux:

``` bash
sudo apt update && sudo apt upgrade -y
sudo apt-get install python3
sudo apt install python3-pip python3-venv git
```

You must also have configured a [base component](https://docs.viam.com/components/base/) and a [vision service detector](https://docs.viam.com/services/vision/detection/). A [power sensor](https://docs.viam.com/components/power-sensor/) is optional and is only used to confirm that charging started.

## API

The detection-dock resource implements the [viam-labs action API](https://github.com/viam-labs/action-api).

Please use the API codebase to interact with a configured version of this service via Viam SDK.

## Viam Service Configuration

The following attributes may be configured as detection dock service config attributes.

For example: the following configuration uses "my_camera" and "my_dock_feature_detector" to find a "match", and drives "my_base" to it. Reaching that target is success. Add `power_sensor` only if a voltage rise should confirm charging.

```json
{
    "base": "my_base",
    "camera": "my_camera",
    "detector": "my_dock_feature_detector",
    "detection_class": "match"
}
```

`base`, `camera`, and `detector` are required implicit dependencies. `power_sensor` is optional.

We used a [Viam Rover](https://www.viam.com/resources/rover) for a base, but other bases can be used. The base must support `SetVelocity`.

### base

*string (required)*

The name of the configured [base component](https://docs.viam.com/components/base/)

### camera

*string (required)*

The name of the configured [camera component](https://docs.viam.com/components/camera/). If `get_images` also returns an `image/vnd.viam.dep` depth frame, it is used to square the robot to the dock surface. That depth frame should be aligned with the color image.

### detector

*string (required)*

The name of the configured [vision service detector](https://docs.viam.com/services/vision/detection/)

### depth_camera

*string (optional)*

A separate [camera](https://docs.viam.com/components/camera/) to read depth from, when the color camera does not include a depth image. Ignored when the color camera already returns depth.

### power_sensor

*string (optional)*

The name of a configured [power sensor](https://docs.viam.com/components/power-sensor/). When set, docking succeeds only if voltage rises by `charge_voltage_delta` after the visual approach. When omitted, the visual approach alone is success.

### detection_class

*string (default: "match")*

Detection class label to track. If several match, the largest box is used.

### camera_fov_deg

*float (default: 70)*

Horizontal camera field of view, in degrees. Used to turn the detection's horizontal offset into a bearing.

### close_percent

*float (default: 0.45)*

Detection width, as a fraction of the image, that counts as having reached the dock.

### center_tolerance

*float (default: 0.08)*

How far the detection center may sit from the image center, as a fraction of image width, and still count as aligned. `0.08` is 8 percent, about 6 degrees at the default 70 degree field of view. An error inside this band does not command a turn during the final adjustments.

### surface_yaw_tolerance_deg

*float (default: 5)*

How many degrees the dock surface may tilt, left versus right in the depth image, and still count as square. Used only when depth is available and the detection is already wide. A tilt past 20 degrees is ignored, as is a single frame that jumps by more than that from the recent median. `bearing_deg` stays the image-center bearing and can read 0 while this tilt is still large.

### docking_distance

*float (default: 0.30)*

Meters. Range is estimated from detection size so that a detection of width `close_percent` is this far away. This scales the approach controller; it is not a measured distance.

### k_phi, k_delta, beta, lambda

*float (defaults: 3.0, 2.0, 0.4, 2.0)*

Gains for the Nav2 graceful control law. `k_phi` pulls the heading onto the line of sight. `k_delta` pulls the robot onto the target heading. `beta` and `lambda` slow the robot when curvature is high.

### v_linear_min, v_linear_max

*float (defaults: 80, 150)*

Approach speed limits, in mm/s.

### v_angular_max

*float (default: 45)*

Maximum angular speed during the approach, in deg/s. Inside the slowdown distance this is replaced by `close_angular_max`.

### close_angular_max

*float (default: 12)*

Maximum turn rate, in deg/s, once the estimated range is within `slowdown_radius` of the dock (at least about 6 inches), and during the final micro-adjustments. The command eases toward that rate so it cannot reverse in one control cycle. During the final adjustments it also waits, stopped, until the bearing holds the new direction before reversing, and the commanded rate is half the heading error.

### search_angular_velocity

*float (default: 10)*

Spin speed while searching for the dock, in deg/s. Used only after `search_settle`.

### search_settle

*float (default: 2)*

Seconds to hold still and keep checking for a detection before the search spin starts.

### search_spin_deg

*float (default: 720)*

How far to spin while searching, in degrees, before giving up. `720` is two full rotations.

### slowdown_radius

*float (default: 0.25)*

Meters. Linear speed scales down inside this distance to the approach goal.

### deceleration_max

*float (default: 1.0)*

Maximum deceleration, in m/s², used to limit approach speed.

### controller_frequency

*float (default: 20)*

How often to grab an image and ask the detector, in Hz. `20` is every 50 ms. A slow detector can only run as fast as it returns.

### initial_perception_timeout

*float (default: 120)*

Safety limit, in seconds, for the search. The search normally ends at `search_spin_deg` first. At the default 10 deg/s, 720 degrees takes about 72 seconds, plus the 2 second settle.

### dock_approach_timeout

*float (default: 30)*

Seconds allowed for one approach.

### external_detection_timeout

*float (default: 1.0)*

Seconds the dock may leave the image during an approach before that attempt fails.

### max_retries

*integer (default: 3)*

Extra attempts after the first. A failed approach backs up by `backup_distance_mm` and tries again.

### backup_distance_mm

*integer (default: 300)*

How far to reverse, in millimeters, before a retry. Driven by holding the approach speed backward for that distance. `move_straight` stops short near the dock.

### wait_charge_timeout

*float (default: 5)*

Seconds to wait for a voltage rise after the visual approach. Used only when `power_sensor` is set.

### charge_voltage_delta

*float (default: 0.12)*

Voltage increase, in volts, that counts as charging. Used only when `power_sensor` is set.

### filter_coef

*float (default: 0.45)*

Exponential smoothing weight for the detected pose, from 0 to 1. Higher values trust the latest detection more.

### attempt_history

*integer (default: 8)*

How many finished docking attempts to keep. Each retry counts as an attempt. `0` keeps none. The attempt in progress is still reported while it runs.

`status` reports `is_running`, `is_docked`, `state` (`idle`, `searching`, `approaching`, `backing_up`, `waiting_charge`, `docked`, `failed`), `retry_count`, `bearing_deg`, `surface_yaw_deg`, `using_depth`, `relative_size`, and `attempts`.

`attempts` is newest first. Each entry has `run` (one `start` call), `retry` (0-based try within that run), `started_at`, `result`, and `steps`. `result` is `running` or why the try ended: `docked`, `stopped`, `search timed out`, `search finished without a detection`, `lost dock detection`, `charging not detected`, or `approach timed out` followed by the bearing, size, and surface yaw that were still short of the goal. Each step has `t_s` (seconds from the start of that try) and `step`, plus pose fields when they matter (`bearing_deg`, `relative_size`, `surface_yaw_deg`, `linear_mm_s`, `angular_deg_s`).

## Troubleshooting

Read `status.attempts` after a run. The first entry is the latest try, and its `result` says why it stopped. `steps` shows the sequence that got there. An approach timeout includes the bearing, detection size, and surface yaw that were still outside tolerance.
