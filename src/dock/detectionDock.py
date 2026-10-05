import asyncio
import math
import time
from array import array
from collections import deque
from dataclasses import dataclass
from typing import Any, ClassVar, Dict, List, Mapping, Optional, Sequence, Tuple, cast

from typing_extensions import Self
from viam.components.base import Base
from viam.components.camera import Camera
from viam.components.power_sensor import PowerSensor
from viam.logging import getLogger
from viam.media.video import CameraMimeType
from viam.module.types import Reconfigurable
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import ResourceName, Vector3
from viam.resource.base import ResourceBase
from viam.resource.types import Model, ModelFamily
from viam.services.vision import VisionClient

from action_python import Action

LOGGER = getLogger(__name__)


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def ego_polar(x: float, y: float, yaw: float) -> Tuple[float, float, float]:
    """Target pose in the robot frame as Nav2 egocentric polar coordinates.

    r is range, phi is the target heading relative to the line of sight, and
    delta is the robot heading relative to the line of sight. Current pose is
    the origin.
    """
    line_of_sight = math.atan2(-y, x)
    return math.hypot(x, y), _wrap(yaw + line_of_sight), _wrap(line_of_sight)


def smooth_velocity(
    r: float,
    phi: float,
    delta: float,
    *,
    k_phi: float,
    k_delta: float,
    beta: float,
    lambda_: float,
    slowdown_radius: float,
    deceleration_max: float,
    v_linear_min: float,
    v_linear_max: float,
    v_angular_max: float,
) -> Tuple[float, float]:
    """Nav2 graceful-controller law. Returns linear m/s and angular rad/s."""
    if r < 1e-4:
        return 0.0, 0.0

    prop_term = k_delta * (delta - math.atan(-k_phi * phi))
    feedback_term = (1.0 + (k_phi / (1.0 + (k_phi * phi) ** 2))) * math.sin(delta)
    curvature = -(prop_term + feedback_term) / r

    v = v_linear_max / (1.0 + beta * abs(curvature) ** lambda_)
    if slowdown_radius > 0:
        v = min(v_linear_max * (r / slowdown_radius), v)
    v = min(math.sqrt(max(0.0, 2.0 * r * deceleration_max)), v)
    v = min(max(v, v_linear_min), v_linear_max)

    if curvature == 0.0:
        return v, 0.0

    w = min(max(curvature * v, -v_angular_max), v_angular_max)
    v = w / curvature
    return v, w


def _raw_depth(image) -> Optional[Tuple[array, int, int]]:
    """Decode an image/vnd.viam.dep frame into millimeters, row-major."""
    if image.mime_type != CameraMimeType.VIAM_RAW_DEPTH:
        return None
    data = image.data
    if len(data) < 24:
        return None
    width = int.from_bytes(data[8:16], "big")
    height = int.from_bytes(data[16:24], "big")
    if width <= 0 or height <= 0:
        return None
    payload = data[24 : 24 + width * height * 2]
    if len(payload) < width * height * 2:
        return None
    depths = array("H")
    depths.frombytes(payload)
    depths.byteswap()
    return depths, width, height


def _depth_strip(depths: array, width: int, height: int, x0: int, x1: int, y0: int, y1: int) -> List[int]:
    values: List[int] = []
    x0 = max(0, x0)
    x1 = min(width, x1)
    y0 = max(0, y0)
    y1 = min(height, y1)
    for y in range(y0, y1):
        row = y * width
        for x in range(x0, x1):
            depth = depths[row + x]
            if 0 < depth < 10000:
                values.append(depth)
    return values


def surface_yaw_rad(
    depth_image,
    x_min: float,
    y_min: float,
    x_max: float,
    y_max: float,
    color_width: int,
    color_height: int,
    fov_deg: float,
) -> Optional[float]:
    """Yaw, in radians, to become perpendicular to the surface inside the detection.

    Positive means turn left. A centered detection can still be oblique; depth on
    the left and right of the box exposes that tilt. Returns None when the frame
    is not depth or there are not enough valid samples.
    """
    decoded = _raw_depth(depth_image)
    if decoded is None or color_width <= 0 or color_height <= 0:
        return None
    depths, depth_width, depth_height = decoded
    scale_x = depth_width / color_width
    scale_y = depth_height / color_height
    left = int(x_min * scale_x)
    right = int(x_max * scale_x)
    top = int(y_min * scale_y)
    bottom = int(y_max * scale_y)
    span = right - left
    band = bottom - top
    if span < 6 or band < 2:
        return None

    mid_top = top + band // 4
    mid_bottom = bottom - band // 4
    if mid_bottom <= mid_top:
        mid_top, mid_bottom = top, bottom
    quarter = max(span // 4, 1)
    left_depths = _depth_strip(depths, depth_width, depth_height, left, left + quarter, mid_top, mid_bottom)
    right_depths = _depth_strip(depths, depth_width, depth_height, right - quarter, right, mid_top, mid_bottom)
    if len(left_depths) < 5 or len(right_depths) < 5:
        return None

    left_depth = _median(left_depths)
    right_depth = _median(right_depths)
    mean_depth = (left_depth + right_depth) / 2.0
    fov = math.radians(fov_deg)

    def camera_x(pixel: float) -> float:
        bearing = ((pixel / depth_width) - 0.5) * fov
        return mean_depth * math.tan(bearing)

    baseline = camera_x(right - quarter / 2.0) - camera_x(left + quarter / 2.0)
    if baseline <= 1.0:
        return None
    # Nose already left of the surface puts the left side farther away.
    # Positive is the correction, turn left, so that tilt has to be negated.
    return math.atan2(right_depth - left_depth, baseline)


def _median(values: List[int]) -> float:
    ordered = sorted(values)
    mid = len(ordered) // 2
    if len(ordered) % 2:
        return float(ordered[mid])
    return (ordered[mid - 1] + ordered[mid]) / 2.0


class _AngleMedian:
    """Median of recent angles, ignoring a spike far from the current value.

    Depth on the two sides of a close detection can disagree for a frame and
    report a tilt the base cannot have rotated through. Those samples must not
    steer.
    """

    def __init__(self, window: int = 11, jump_rad: float = math.radians(20.0)):
        self._samples: deque = deque(maxlen=window)
        self._value: Optional[float] = None
        self._jump = jump_rad

    def add(self, sample: float) -> float:
        # A dock face cannot jump tens of degrees in one frame. Keep the last
        # median instead of adopting the spike.
        if self._value is not None and abs(_wrap(sample - self._value)) > self._jump:
            return self._value
        self._samples.append(sample)
        ordered = sorted(self._samples)
        self._value = ordered[len(ordered) // 2]
        return self._value


@dataclass
class _TurnSettle:
    committed: int = 0
    pending: int = 0
    pending_since: Optional[float] = None


def _settled_turn(
    angular: float,
    now: float,
    state: _TurnSettle,
    settle_s: float,
    moving: bool,
) -> Tuple[float, bool]:
    """Hold a stop until `angular` keeps one direction through `settle_s` of stillness.

    A detection that jumps across center would otherwise reverse the spin every
    cycle. Returns the command to send and whether that command is still waiting.
    """
    sign = 0 if abs(angular) < 1e-4 else (1 if angular > 0 else -1)
    if sign == 0:
        state.committed = 0
        state.pending = 0
        state.pending_since = None
        return 0.0, False
    if sign == state.committed:
        state.pending = sign
        state.pending_since = None
        return angular, False
    if state.pending != sign:
        state.pending = sign
        state.pending_since = None
        return 0.0, True
    if moving:
        state.pending_since = None
        return 0.0, True
    if state.pending_since is None:
        state.pending_since = now
        return 0.0, True
    if now - state.pending_since < settle_s:
        return 0.0, True
    state.committed = sign
    state.pending_since = None
    return angular, False


def _color_and_depth(images):
    color = None
    depth = None
    for image in images:
        if image.mime_type == CameraMimeType.VIAM_RAW_DEPTH:
            if depth is None:
                depth = image
        elif image.mime_type != CameraMimeType.PCD and color is None:
            color = image
    return color, depth


@dataclass
class Sighting:
    center_offset: float
    relative_size: float
    surface_yaw: Optional[float]


@dataclass
class Status:
    is_running: bool = False
    is_docked: bool = False
    state: str = "idle"
    retry_count: int = 0
    bearing_deg: float = 0.0
    relative_size: float = 0.0
    surface_yaw_deg: float = 0.0
    using_depth: bool = False


class detectionDock(Action, Reconfigurable):
    MODEL: ClassVar[Model] = Model(ModelFamily("viam-labs", "dock"), "detection-dock")

    power_sensor: Optional[PowerSensor]
    depth_camera: Optional[Camera]
    base: Base
    camera: Camera
    detector: VisionClient
    internal_status: Status

    @classmethod
    def new(cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]) -> Self:
        my_class = cls(config.name)
        my_class.internal_status = Status()
        my_class.power_sensor = None
        my_class.depth_camera = None
        my_class._attempts = deque()
        my_class._run_serial = 0
        my_class._current_attempt = None
        my_class._attempt_t0 = 0.0
        my_class._attempt_result = ""
        my_class.attempt_history = 8
        my_class.reconfigure(config, dependencies)
        return my_class

    @classmethod
    def validate(cls, config: ComponentConfig) -> Tuple[Sequence[str], Sequence[str]]:
        fields = config.attributes.fields

        def required(name: str) -> str:
            value = fields[name].string_value if name in fields else ""
            if value == "":
                raise Exception(f"{name} must be defined")
            return value

        base = required("base")
        camera = required("camera")
        detector = required("detector")
        optional = []
        for name in ("power_sensor", "depth_camera"):
            value = fields[name].string_value if name in fields else ""
            if value:
                optional.append(value)
        return [base, camera, detector], optional

    def reconfigure(self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]):
        fields = config.attributes.fields

        base = fields["base"].string_value
        self.base = cast(Base, dependencies[Base.get_resource_name(base)])

        camera = fields["camera"].string_value
        self.camera = cast(Camera, dependencies[Camera.get_resource_name(camera)])

        detector = fields["detector"].string_value
        self.detector = cast(VisionClient, dependencies[VisionClient.get_resource_name(detector)])

        power_sensor = fields["power_sensor"].string_value if "power_sensor" in fields else ""
        self.power_sensor = None
        if power_sensor:
            power_name = PowerSensor.get_resource_name(power_sensor)
            if power_name in dependencies:
                self.power_sensor = cast(PowerSensor, dependencies[power_name])
            else:
                LOGGER.warning(
                    "power_sensor %s is not available; docking will succeed on visual arrival",
                    power_sensor,
                )

        depth_camera = fields["depth_camera"].string_value if "depth_camera" in fields else ""
        self.depth_camera = None
        if depth_camera:
            depth_name = Camera.get_resource_name(depth_camera)
            if depth_name in dependencies:
                self.depth_camera = cast(Camera, dependencies[depth_name])
            else:
                LOGGER.warning("depth_camera %s is not available; docking will use image bearing only", depth_camera)

        self.detection_class = _string(fields, "detection_class", "match")
        self.camera_fov_deg = _number(fields, "camera_fov_deg", 70.0)
        self.close_percent = _number(fields, "close_percent", 0.45)
        self.center_tolerance = _number(fields, "center_tolerance", 0.08)
        self.surface_yaw_tolerance = math.radians(_number(fields, "surface_yaw_tolerance_deg", 5.0))
        self.docking_distance = _number(fields, "docking_distance", 0.30)

        self.k_phi = _number(fields, "k_phi", 3.0)
        self.k_delta = _number(fields, "k_delta", 2.0)
        self.beta = _number(fields, "beta", 0.4)
        self.lambda_ = _number(fields, "lambda", 2.0)
        self.slowdown_radius = _number(fields, "slowdown_radius", 0.25)
        self.deceleration_max = _number(fields, "deceleration_max", 1.0)
        self.v_linear_min = _number(fields, "v_linear_min", 80.0) / 1000.0
        self.v_linear_max = _number(fields, "v_linear_max", 150.0) / 1000.0
        self.v_angular_max = math.radians(_number(fields, "v_angular_max", 45.0))
        # Near the dock the curvature term saturates and then reverses, which
        # is the violent swing. Cap the turn on the way in. Once inside
        # micro_distance, abandon that law and creep forward, back, or in yaw.
        self.close_angular_max = math.radians(_number(fields, "close_angular_max", 12.0))
        self.close_angular_slew = math.radians(30.0)
        self.micro_distance = 0.12
        self.micro_linear = min(self.v_linear_min, 0.04)
        # Short reverse while squaring up. Long enough to swing the nose
        # without the dock leaving the camera, short enough not to hit
        # whatever is behind the base.
        self.square_backup_m = 0.10
        # Close in, a noisy box center makes bearing flip every cycle. Filter
        # harder, and do not reverse the spin until the base has been still
        # long enough for that filtered bearing to stay on one side.
        self.close_filter_coef = 0.08
        self.bearing_settle = 0.5
        self.search_angular_velocity = math.radians(_number(fields, "search_angular_velocity", 10.0))
        self.search_spin_deg = _number(fields, "search_spin_deg", 720.0)
        self.search_settle = _number(fields, "search_settle", 2.0)

        self.controller_frequency = _number(fields, "controller_frequency", 20.0)
        self.initial_perception_timeout = _number(fields, "initial_perception_timeout", 120.0)
        self.dock_approach_timeout = _number(fields, "dock_approach_timeout", 30.0)
        self.external_detection_timeout = _number(fields, "external_detection_timeout", 1.0)
        self.wait_charge_timeout = _number(fields, "wait_charge_timeout", 5.0)
        self.charge_voltage_delta = _number(fields, "charge_voltage_delta", 0.12)
        self.max_retries = int(_number(fields, "max_retries", 3, allow_zero=True))
        self.backup_distance_mm = int(_number(fields, "backup_distance_mm", 300, allow_zero=True))
        self.filter_coef = min(max(_number(fields, "filter_coef", 0.45), 0.01), 1.0)
        self.attempt_history = max(0, int(_number(fields, "attempt_history", 8, allow_zero=True)))

        if not hasattr(self, "internal_status"):
            self.internal_status = Status()
        if not hasattr(self, "_run_serial"):
            self._run_serial = 0
            self._current_attempt = None
            self._attempt_t0 = 0.0
            self._attempt_result = ""
        kept = list(getattr(self, "_attempts", ()))
        if self.attempt_history <= 0:
            self._attempts = deque()
        else:
            self._attempts = deque(kept[-self.attempt_history :], maxlen=self.attempt_history)

    async def dock(self):
        self.internal_status.is_running = True
        self.internal_status.is_docked = False
        self.internal_status.retry_count = 0
        self.internal_status.using_depth = False
        self.internal_status.surface_yaw_deg = 0.0
        self.internal_status.state = "searching"
        self._run_serial += 1

        try:
            for retry in range(self.max_retries + 1):
                if not self.internal_status.is_running:
                    break
                self.internal_status.retry_count = retry
                self._begin_attempt(retry)
                self._log_step("searching")

                found = await self._acquire()
                if not self.internal_status.is_running:
                    self._log_step("stopped")
                    self._finish_attempt("stopped")
                    break
                if not found:
                    self._finish_attempt(self._attempt_result or "dock not detected")
                    break

                approached = await self._approach()
                if not self.internal_status.is_running:
                    self._log_step("stopped")
                    self._finish_attempt("stopped")
                    break
                if approached:
                    if self.power_sensor is None:
                        self._log_step("docked")
                        self._finish_attempt("docked")
                        self._mark_docked()
                        return
                    self.internal_status.state = "waiting_charge"
                    if await self._wait_for_charge():
                        self._log_step("docked")
                        self._finish_attempt("docked")
                        self._mark_docked()
                        return
                    self._attempt_result = "charging not detected"

                if not self.internal_status.is_running:
                    self._log_step("stopped")
                    self._finish_attempt("stopped")
                    break
                await self._backup()
                self._finish_attempt(self._attempt_result or "approach failed")

            if not self.internal_status.is_docked:
                self.internal_status.state = "idle" if not self.internal_status.is_running else "failed"
        except Exception as exc:
            self._attempt_result = f"error: {exc}"
            self._log_step("error", message=str(exc))
            LOGGER.exception("docking failed")
            raise
        finally:
            if self._current_attempt is not None:
                self._finish_attempt(self._attempt_result or "stopped")
            self.internal_status.is_running = False
            try:
                await self.base.stop()
            except Exception:
                LOGGER.exception("failed to stop base")

    def _begin_attempt(self, retry: int):
        self._attempt_t0 = time.monotonic()
        self._attempt_result = ""
        self._current_attempt = {
            "run": self._run_serial,
            "retry": retry,
            "started_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "result": "running",
            "steps": [],
        }

    def _log_step(self, step: str, **fields: Any):
        current = self._current_attempt
        if current is None:
            return
        record: Dict[str, Any] = {"t_s": round(time.monotonic() - self._attempt_t0, 2), "step": step}
        for key, value in fields.items():
            if value is None:
                continue
            record[key] = _log_value(key, value)
        steps: List[Dict[str, Any]] = current["steps"]
        steps.append(record)
        if len(steps) > 80:
            del steps[: len(steps) - 80]
        detail = " ".join(f"{key}={record[key]}" for key in record if key not in ("t_s", "step"))
        if detail:
            LOGGER.info("dock run %s retry %s %s %s", current["run"], current["retry"], step, detail)
        else:
            LOGGER.info("dock run %s retry %s %s", current["run"], current["retry"], step)

    def _finish_attempt(self, result: str):
        current = self._current_attempt
        if current is None:
            return
        current["result"] = result
        self._current_attempt = None
        if self.attempt_history > 0:
            self._attempts.append(current)
        LOGGER.info("dock run %s retry %s finished: %s", current["run"], current["retry"], result)

    def _pose_fields(self, center: Optional[float], size: Optional[float], yaw: Optional[float]) -> Dict[str, float]:
        fields: Dict[str, float] = {}
        if size is not None:
            fields["relative_size"] = size
        if center is not None:
            fields["bearing_deg"] = -center * self.camera_fov_deg
        if yaw is not None:
            fields["surface_yaw_deg"] = math.degrees(yaw)
        return fields

    def _alignment_gap(self, size: Optional[float], center: Optional[float], yaw: Optional[float]) -> str:
        if size is None or center is None:
            return "no detection"
        gaps = []
        if size < self.close_percent:
            gaps.append(f"size {size:.2f} below {self.close_percent:.2f}")
        bearing_deg = -center * self.camera_fov_deg
        bearing_limit = self.center_tolerance * self.camera_fov_deg
        if abs(center) > self.center_tolerance:
            gaps.append(f"bearing {bearing_deg:.1f} deg outside {bearing_limit:.1f}")
        if yaw is not None and abs(yaw) > self.surface_yaw_tolerance:
            gaps.append(
                f"surface yaw {math.degrees(yaw):.1f} deg outside {math.degrees(self.surface_yaw_tolerance):.1f}"
            )
        if not gaps:
            return "in tolerance but not held"
        return "; ".join(gaps)

    def _mark_docked(self):
        self.internal_status.is_docked = True
        self.internal_status.state = "docked"

    async def _acquire(self) -> bool:
        self.internal_status.state = "searching"
        await self.base.stop()
        settle_until = time.monotonic() + self.search_settle
        speed_deg = abs(math.degrees(self.search_angular_velocity))
        started_spin = None
        exit_reason = "search timed out"
        deadline = time.monotonic() + self.initial_perception_timeout
        while self.internal_status.is_running and time.monotonic() < deadline:
            loop_start = time.monotonic()
            if started_spin is not None and speed_deg * (time.monotonic() - started_spin) >= self.search_spin_deg:
                exit_reason = "search finished without a detection"
                break
            sample = await self._detect()
            if sample is not None:
                yaw = None if sample.surface_yaw is None else math.degrees(sample.surface_yaw)
                self._log_step(
                    "dock_detected",
                    bearing_deg=-sample.center_offset * self.camera_fov_deg,
                    relative_size=sample.relative_size,
                    surface_yaw_deg=yaw,
                )
                await self.base.stop()
                return True
            if time.monotonic() >= settle_until:
                if started_spin is None:
                    started_spin = time.monotonic()
                    self._log_step("search_spin", angular_deg_s=math.degrees(self.search_angular_velocity))
                await self._command(0.0, self.search_angular_velocity)
            elapsed = time.monotonic() - loop_start
            await asyncio.sleep(max(0.0, (1.0 / self.controller_frequency) - elapsed))
        await self.base.stop()
        if not self.internal_status.is_running:
            return False
        self._attempt_result = exit_reason
        self._log_step("search_exhausted" if exit_reason.startswith("search finished") else "search_timed_out")
        return False

    async def _approach(self) -> bool:
        self.internal_status.state = "approaching"
        self._log_step("approaching")
        deadline = time.monotonic() + self.dock_approach_timeout
        last_seen = time.monotonic()
        filtered_center: Optional[float] = None
        filtered_size: Optional[float] = None
        filtered_yaw: Optional[float] = None
        last_center: Optional[float] = None
        last_size: Optional[float] = None
        last_yaw: Optional[float] = None
        commanded_linear = 0.0
        commanded_angular = 0.0
        micro = False
        smooth_close = False
        turn_state = _TurnSettle()
        holding_turn = False
        aligned_since: Optional[float] = None
        yaw_filter = _AngleMedian()
        square_reversed = 0.0
        logged_square = False
        command_at: Optional[float] = None

        while self.internal_status.is_running and time.monotonic() < deadline:
            loop_start = time.monotonic()
            if command_at is not None and commanded_linear < 0:
                square_reversed += -commanded_linear * (loop_start - command_at)
            sample = await self._detect()
            if sample is None:
                if time.monotonic() - last_seen > self.external_detection_timeout:
                    because = (
                        "detection never returned"
                        if last_center is None
                        else self._alignment_gap(last_size, last_center, last_yaw)
                    )
                    self._attempt_result = "lost dock detection"
                    self._log_step(
                        "lost_dock_detection",
                        because=because,
                        linear_mm_s=commanded_linear * 1000.0,
                        angular_deg_s=math.degrees(commanded_angular),
                        **self._pose_fields(last_center, last_size, last_yaw),
                    )
                    await self.base.stop()
                    return False
            else:
                now = time.monotonic()
                last_seen = now
                center_offset = sample.center_offset
                relative_size = sample.relative_size
                coef = min(self.filter_coef, self.close_filter_coef) if smooth_close else self.filter_coef
                if filtered_center is None or filtered_size is None:
                    filtered_center, filtered_size = center_offset, relative_size
                else:
                    filtered_center = (1.0 - coef) * filtered_center + coef * center_offset
                    filtered_size = (1.0 - coef) * filtered_size + coef * relative_size

                if sample.surface_yaw is not None:
                    if not self.internal_status.using_depth:
                        self._log_step("using_depth")
                    self.internal_status.using_depth = True
                    filtered_yaw = yaw_filter.add(sample.surface_yaw)
                    self.internal_status.surface_yaw_deg = math.degrees(filtered_yaw)

                psi = -filtered_center * math.radians(self.camera_fov_deg)
                distance = self.docking_distance * (self.close_percent / max(filtered_size, 1e-3))
                self.internal_status.bearing_deg = math.degrees(psi)
                self.internal_status.relative_size = filtered_size
                last_center, last_size, last_yaw = filtered_center, filtered_size, filtered_yaw

                control_yaw = self._trusted_yaw(filtered_size, filtered_yaw)
                squared = control_yaw is None or abs(control_yaw) <= self.surface_yaw_tolerance
                aligned = (
                    filtered_size >= self.close_percent
                    and abs(filtered_center) <= self.center_tolerance
                    and squared
                )
                moving = abs(commanded_angular) > math.radians(2.0) or abs(commanded_linear) > 0.015
                if aligned and not moving:
                    if aligned_since is None:
                        aligned_since = now
                        self._log_step(
                            "holding_alignment",
                            **self._pose_fields(filtered_center, filtered_size, filtered_yaw),
                        )
                    elif now - aligned_since >= self.bearing_settle:
                        self._log_step(
                            "aligned",
                            **self._pose_fields(filtered_center, filtered_size, filtered_yaw),
                        )
                        await self.base.stop()
                        return True
                else:
                    aligned_since = None

                # The graceful controller's goal yaw is opposite the usual left-positive heading.
                # Inside micro_distance that law stalls, because forward speed collapses with
                # curvature. Creep forward, back, or in yaw instead. Stay in that mode until
                # the dock is clearly farther again, so a small reverse does not hand control back.
                remaining = distance - self.docking_distance
                if remaining <= self.micro_distance:
                    if not micro:
                        self._log_step(
                            "micro_adjusting",
                            **self._pose_fields(filtered_center, filtered_size, filtered_yaw),
                        )
                    micro = True
                elif remaining > self.micro_distance + 0.08:
                    micro = False

                if micro:
                    linear, angular = self._micro_velocity(
                        psi, filtered_size, control_yaw, square_reversed
                    )
                    # The reverse is what keeps the dock in frame while the
                    # heading changes. The settle hold would cancel it.
                    if linear < 0:
                        if not logged_square:
                            self._log_step(
                                "squaring_up",
                                distance_mm=int(self.square_backup_m * 1000),
                                **self._pose_fields(filtered_center, filtered_size, filtered_yaw),
                            )
                            logged_square = True
                        holding_turn = False
                        turn_state.committed = 0
                        turn_state.pending = 0
                        turn_state.pending_since = None
                    else:
                        angular, holding = _settled_turn(angular, now, turn_state, self.bearing_settle, moving)
                        if holding and not holding_turn:
                            self._log_step(
                                "holding_bearing",
                                linear_mm_s=commanded_linear * 1000.0,
                                angular_deg_s=math.degrees(commanded_angular),
                                **self._pose_fields(filtered_center, filtered_size, filtered_yaw),
                            )
                        holding_turn = holding
                        if holding or aligned:
                            linear = 0.0
                else:
                    turn_state.committed = 0
                    turn_state.pending = 0
                    turn_state.pending_since = None
                    holding_turn = False
                    # Head toward the dock. Surface tilt at long range sits near
                    # -27 deg no matter the image bearing, so steering on it
                    # turns the dock out of frame before the base gets close.
                    linear, angular = self._approach_velocity(psi, distance, psi)
                    if remaining <= max(self.slowdown_radius, 0.15):
                        angular = max(-self.close_angular_max, min(self.close_angular_max, angular))

                if micro or remaining <= max(self.slowdown_radius, 0.15):
                    step = self.close_angular_slew / max(self.controller_frequency, 1.0)
                    angular = commanded_angular + max(-step, min(step, angular - commanded_angular))
                if micro:
                    linear_step = 0.15 / max(self.controller_frequency, 1.0)
                    linear = commanded_linear + max(-linear_step, min(linear_step, linear - commanded_linear))
                commanded_linear = linear
                commanded_angular = angular
                command_at = time.monotonic()
                smooth_close = micro
                await self._command(linear, angular)

            elapsed = time.monotonic() - loop_start
            await asyncio.sleep(max(0.0, (1.0 / self.controller_frequency) - elapsed))

        await self.base.stop()
        if not self.internal_status.is_running:
            return False
        because = self._alignment_gap(last_size, last_center, last_yaw)
        self._attempt_result = f"approach timed out: {because}"
        self._log_step(
            "approach_timed_out",
            because=because,
            linear_mm_s=commanded_linear * 1000.0,
            angular_deg_s=math.degrees(commanded_angular),
            **self._pose_fields(last_center, last_size, last_yaw),
        )
        return False

    def _trusted_yaw(self, size: Optional[float], yaw: Optional[float]) -> Optional[float]:
        """Yaw worth steering on. A narrow box, or a tilt past 20 deg, is not.

        Far from the dock the left and right depth samples are a few centimeters
        apart, so a small depth bias reads as about -27 deg on every approach.
        Up close, a bad pair of samples reads as 45 deg or more and flips sign.
        Neither should turn the base.
        """
        if size is None or yaw is None or size < 0.35:
            return None
        if abs(yaw) > math.radians(20.0):
            return None
        return yaw

    def _micro_velocity(
        self,
        psi: float,
        filtered_size: float,
        filtered_yaw: Optional[float],
        reversed_m: float,
    ) -> Tuple[float, float]:
        """Small correction once the dock is close. Heading is left-positive.

        Driving forward while angled runs a corner into obstacle avoidance and
        the base stops. Spinning in place swings the dock out of the camera.
        Back up a short distance instead, and only turn while the dock is still
        near the center so the heading change is kept as the target settles back.
        """
        bearing_tol = self.center_tolerance * math.radians(self.camera_fov_deg)
        yaw_err = 0.0 if filtered_yaw is None else filtered_yaw
        bearing_outside = abs(psi) > bearing_tol
        yaw_outside = filtered_yaw is not None and abs(yaw_err) > self.surface_yaw_tolerance
        turn = min(self.close_angular_max, math.radians(10.0))
        if yaw_outside:
            dock_centered = abs(psi) <= bearing_tol * 0.5
            angular = math.copysign(turn, yaw_err) if dock_centered else 0.0
            if reversed_m < self.square_backup_m:
                return -self.v_linear_min, angular
            if dock_centered:
                return 0.0, angular
            return 0.0, math.copysign(turn, psi)
        if bearing_outside:
            return 0.0, math.copysign(turn, psi)

        if filtered_size >= self.close_percent:
            over = filtered_size - self.close_percent
            if over > 0.08:
                return -self.micro_linear * min(1.0, (over - 0.08) / 0.10), 0.0
            return 0.0, 0.0

        size_error = self.close_percent - filtered_size
        linear = self.micro_linear * max(0.0, min(1.0, size_error / 0.10))
        if size_error > 0.01:
            linear = max(linear, 0.03)
        return linear, 0.0

    def _approach_velocity(self, psi: float, distance: float, goal_yaw: float) -> Tuple[float, float]:
        goal_dist = max(distance - self.docking_distance, 1e-3)
        r, phi, delta = ego_polar(goal_dist * math.cos(psi), goal_dist * math.sin(psi), goal_yaw)
        return smooth_velocity(
            r,
            phi,
            delta,
            k_phi=self.k_phi,
            k_delta=self.k_delta,
            beta=self.beta,
            lambda_=self.lambda_,
            slowdown_radius=self.slowdown_radius,
            deceleration_max=self.deceleration_max,
            v_linear_min=self.v_linear_min,
            v_linear_max=self.v_linear_max,
            v_angular_max=self.v_angular_max,
        )

    async def _detect(self) -> Optional[Sighting]:
        images, _ = await self.camera.get_images()
        if not images:
            return None
        color, depth = _color_and_depth(images)
        if color is None:
            return None
        width = color.width or 0
        height = color.height or 0
        if width <= 0 or height <= 0:
            return None

        if depth is None and self.depth_camera is not None:
            depth_images, _ = await self.depth_camera.get_images()
            _, depth = _color_and_depth(depth_images)

        detections = await self.detector.get_detections(color)
        matches = [det for det in detections if not self.detection_class or det.class_name == self.detection_class]
        if not matches:
            return None

        best = max(matches, key=lambda det: (det.x_max - det.x_min) * (det.y_max - det.y_min))
        center_offset = ((best.x_min + best.x_max) / 2.0) / width - 0.5
        relative_size = (best.x_max - best.x_min) / width
        surface_yaw = None
        if depth is not None:
            surface_yaw = surface_yaw_rad(
                depth,
                best.x_min,
                best.y_min,
                best.x_max,
                best.y_max,
                width,
                height,
                self.camera_fov_deg,
            )
        return Sighting(center_offset, relative_size, surface_yaw)

    async def _wait_for_charge(self) -> bool:
        if self.power_sensor is None:
            return True
        start_voltage, _ = await self.power_sensor.get_voltage()
        voltage = start_voltage
        self._log_step("waiting_for_charge", voltage_v=float(start_voltage))
        deadline = time.monotonic() + self.wait_charge_timeout
        while self.internal_status.is_running and time.monotonic() < deadline:
            await asyncio.sleep(0.2)
            voltage, _ = await self.power_sensor.get_voltage()
            if voltage - start_voltage > self.charge_voltage_delta:
                self._log_step(
                    "charge_detected",
                    voltage_v=float(voltage),
                    voltage_delta_v=float(voltage - start_voltage),
                )
                return True
        if not self.internal_status.is_running:
            return False
        self._log_step(
            "charging_not_detected",
            voltage_v=float(voltage),
            voltage_delta_v=float(voltage - start_voltage),
        )
        return False

    async def _backup(self):
        if not self.internal_status.is_running or self.backup_distance_mm <= 0:
            return
        # move_straight stops almost immediately near the dock, so the base
        # barely reverses. Hold the approach velocity backward for the distance.
        speed = self.v_linear_max
        if speed <= 0:
            return
        self.internal_status.state = "backing_up"
        duration = (self.backup_distance_mm / 1000.0) / speed
        self._log_step("backing_up", distance_mm=self.backup_distance_mm)
        try:
            await self._command(-speed, 0.0)
            deadline = time.monotonic() + duration
            while self.internal_status.is_running and time.monotonic() < deadline:
                await asyncio.sleep(0.05)
        finally:
            try:
                await self.base.stop()
            except Exception:
                LOGGER.exception("failed to stop base after backup")

    async def _command(self, linear_mps: float, angular_radps: float):
        await self.base.set_velocity(
            linear=Vector3(x=0, y=linear_mps * 1000.0, z=0),
            angular=Vector3(x=0, y=0, z=math.degrees(angular_radps)),
        )

    async def start(self) -> str:
        if not self.internal_status.is_running:
            asyncio.ensure_future(self.dock())
        return "OK"

    async def stop(self) -> str:
        self.internal_status.is_running = False
        try:
            await self.base.stop()
        except Exception:
            LOGGER.exception("failed to stop base")
        return "OK"

    async def is_running(self) -> bool:
        return self.internal_status.is_running

    def _attempt_view(self, attempt: Mapping[str, Any]) -> Dict[str, Any]:
        return {
            "run": attempt["run"],
            "retry": attempt["retry"],
            "started_at": attempt["started_at"],
            "result": attempt["result"],
            "steps": list(attempt["steps"]),
        }

    async def status(self) -> Mapping[str, Any]:
        status = self.internal_status
        attempts = [self._attempt_view(attempt) for attempt in self._attempts]
        if self._current_attempt is not None:
            attempts.append(self._attempt_view(self._current_attempt))
        attempts.reverse()
        return {
            "is_running": status.is_running,
            "is_docked": status.is_docked,
            "state": status.state,
            "retry_count": status.retry_count,
            "bearing_deg": status.bearing_deg,
            "surface_yaw_deg": status.surface_yaw_deg,
            "using_depth": status.using_depth,
            "relative_size": status.relative_size,
            "attempts": attempts,
        }


def _log_value(key: str, value: Any) -> Any:
    if not isinstance(value, float):
        return value
    if key in ("relative_size", "voltage_v", "voltage_delta_v"):
        return round(value, 3)
    if key.endswith("_deg") or key.endswith("_deg_s"):
        return round(value, 1)
    if key.endswith("_mm_s"):
        return round(value, 1)
    return round(value, 3)


def _string(fields, name: str, default: str) -> str:
    if name not in fields:
        return default
    return fields[name].string_value or default


def _number(fields, name: str, default: float, allow_zero: bool = False) -> float:
    if name not in fields:
        return default
    value = fields[name].number_value
    if value == 0 and not allow_zero:
        return default
    return value
