import asyncio
import math
import time
from array import array
from dataclasses import dataclass
from typing import Any, ClassVar, List, Mapping, Optional, Sequence, Tuple, cast

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
    # Right side farther means the surface faces to the right, so yaw right.
    return math.atan2(left_depth - right_depth, baseline)


def _median(values: List[int]) -> float:
    ordered = sorted(values)
    mid = len(ordered) // 2
    if len(ordered) % 2:
        return float(ordered[mid])
    return (ordered[mid - 1] + ordered[mid]) / 2.0


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
        self.center_tolerance = _number(fields, "center_tolerance", 0.05)
        self.surface_yaw_tolerance = math.radians(_number(fields, "surface_yaw_tolerance_deg", 5.0))
        self.docking_distance = _number(fields, "docking_distance", 0.30)
        self.align_distance = _number(fields, "align_distance", 0.20)
        self.align_near = 0.10
        self.align_nudge = math.radians(_number(fields, "align_nudge_deg", 10.0))

        self.k_phi = _number(fields, "k_phi", 3.0)
        self.k_delta = _number(fields, "k_delta", 2.0)
        self.beta = _number(fields, "beta", 0.4)
        self.lambda_ = _number(fields, "lambda", 2.0)
        self.slowdown_radius = _number(fields, "slowdown_radius", 0.25)
        self.deceleration_max = _number(fields, "deceleration_max", 1.0)
        self.v_linear_min = _number(fields, "v_linear_min", 80.0) / 1000.0
        self.v_linear_max = _number(fields, "v_linear_max", 150.0) / 1000.0
        self.v_angular_max = math.radians(_number(fields, "v_angular_max", 45.0))
        self.search_angular_velocity = math.radians(_number(fields, "search_angular_velocity", 15.0))
        self.search_spin_deg = _number(fields, "search_spin_deg", 720.0)

        self.controller_frequency = _number(fields, "controller_frequency", 8.0)
        self.initial_perception_timeout = _number(fields, "initial_perception_timeout", 120.0)
        self.dock_approach_timeout = _number(fields, "dock_approach_timeout", 30.0)
        self.external_detection_timeout = _number(fields, "external_detection_timeout", 1.0)
        self.wait_charge_timeout = _number(fields, "wait_charge_timeout", 5.0)
        self.charge_voltage_delta = _number(fields, "charge_voltage_delta", 0.12)
        self.max_retries = int(_number(fields, "max_retries", 3, allow_zero=True))
        self.backup_distance_mm = int(_number(fields, "backup_distance_mm", 300, allow_zero=True))
        self.filter_coef = min(max(_number(fields, "filter_coef", 0.45), 0.01), 1.0)

        if not hasattr(self, "internal_status"):
            self.internal_status = Status()

    async def dock(self):
        self.internal_status.is_running = True
        self.internal_status.is_docked = False
        self.internal_status.retry_count = 0
        self.internal_status.using_depth = False
        self.internal_status.surface_yaw_deg = 0.0
        self.internal_status.state = "searching"

        try:
            for attempt in range(self.max_retries + 1):
                if not self.internal_status.is_running:
                    break
                self.internal_status.retry_count = attempt
                LOGGER.info("dock attempt %s", attempt + 1)

                if not await self._acquire():
                    if self.internal_status.is_running:
                        LOGGER.warning("dock not detected")
                    break

                if await self._approach():
                    if self.power_sensor is None:
                        self._mark_docked()
                        return
                    self.internal_status.state = "waiting_charge"
                    if await self._wait_for_charge():
                        self._mark_docked()
                        return
                    LOGGER.info("charging not detected")

                if self.internal_status.is_running:
                    await self._backup()

            if not self.internal_status.is_docked:
                self.internal_status.state = "idle" if not self.internal_status.is_running else "failed"
        finally:
            self.internal_status.is_running = False
            try:
                await self.base.stop()
            except Exception:
                LOGGER.exception("failed to stop base")

    def _mark_docked(self):
        self.internal_status.is_docked = True
        self.internal_status.state = "docked"
        LOGGER.info("docked")

    async def _acquire(self) -> bool:
        self.internal_status.state = "searching"
        speed_deg = abs(math.degrees(self.search_angular_velocity))
        started_spin = None
        deadline = time.monotonic() + self.initial_perception_timeout
        while self.internal_status.is_running and time.monotonic() < deadline:
            if started_spin is not None and speed_deg * (time.monotonic() - started_spin) >= self.search_spin_deg:
                break
            sample = await self._detect()
            if sample is not None:
                await self.base.stop()
                return True
            if started_spin is None:
                started_spin = time.monotonic()
            await self._command(0.0, self.search_angular_velocity)
            await asyncio.sleep(1.0 / self.controller_frequency)
        await self.base.stop()
        return False

    async def _approach(self) -> bool:
        self.internal_status.state = "approaching"
        deadline = time.monotonic() + self.dock_approach_timeout
        last_seen = time.monotonic()
        filtered_center: Optional[float] = None
        filtered_size: Optional[float] = None
        filtered_yaw: Optional[float] = None

        while self.internal_status.is_running and time.monotonic() < deadline:
            loop_start = time.monotonic()
            sample = await self._detect()
            if sample is None:
                await self.base.stop()
                if time.monotonic() - last_seen > self.external_detection_timeout:
                    LOGGER.info("lost dock detection")
                    return False
            else:
                last_seen = time.monotonic()
                center_offset = sample.center_offset
                relative_size = sample.relative_size
                if filtered_center is None or filtered_size is None:
                    filtered_center, filtered_size = center_offset, relative_size
                else:
                    coef = self.filter_coef
                    filtered_center = (1.0 - coef) * filtered_center + coef * center_offset
                    filtered_size = (1.0 - coef) * filtered_size + coef * relative_size

                if sample.surface_yaw is not None:
                    if not self.internal_status.using_depth:
                        LOGGER.info("aligning to the dock surface with depth")
                    self.internal_status.using_depth = True
                    if filtered_yaw is None:
                        filtered_yaw = sample.surface_yaw
                    else:
                        filtered_yaw = _wrap(filtered_yaw + self.filter_coef * _wrap(sample.surface_yaw - filtered_yaw))
                    self.internal_status.surface_yaw_deg = math.degrees(filtered_yaw)

                psi = -filtered_center * math.radians(self.camera_fov_deg)
                distance = self.docking_distance * (self.close_percent / max(filtered_size, 1e-3))
                self.internal_status.bearing_deg = math.degrees(psi)
                self.internal_status.relative_size = filtered_size

                squared = filtered_yaw is None or abs(filtered_yaw) <= self.surface_yaw_tolerance
                if filtered_size >= self.close_percent and abs(filtered_center) <= self.center_tolerance and squared:
                    await self.base.stop()
                    return True

                # Same approach as before the 0.3.5 alignment takeover. Between about
                # 4 and 8 inches short of the dock, add a small turn. It does not replace the drive.
                # The graceful controller's goal yaw is opposite the usual left-positive heading.
                self.internal_status.state = "approaching"
                goal_yaw = -filtered_yaw if filtered_yaw is not None else psi
                linear, angular = self._approach_velocity(psi, distance, goal_yaw)
                remaining = distance - self.docking_distance
                if self.align_near < remaining <= self.align_distance:
                    error = filtered_yaw if filtered_yaw is not None else psi
                    extra = max(-self.align_nudge, min(self.align_nudge, error))
                    angular = max(-self.v_angular_max, min(self.v_angular_max, angular + extra))
                await self._command(linear, angular)

            elapsed = time.monotonic() - loop_start
            await asyncio.sleep(max(0.0, (1.0 / self.controller_frequency) - elapsed))

        await self.base.stop()
        return False

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
        deadline = time.monotonic() + self.wait_charge_timeout
        while self.internal_status.is_running and time.monotonic() < deadline:
            await asyncio.sleep(0.2)
            voltage, _ = await self.power_sensor.get_voltage()
            if voltage - start_voltage > self.charge_voltage_delta:
                return True
        return False

    async def _backup(self):
        if not self.internal_status.is_running:
            return
        await self.base.move_straight(-self.backup_distance_mm, int(self.v_linear_max * 1000))

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

    async def status(self) -> Mapping[str, Any]:
        status = self.internal_status
        return {
            "is_running": status.is_running,
            "is_docked": status.is_docked,
            "state": status.state,
            "retry_count": status.retry_count,
            "bearing_deg": status.bearing_deg,
            "surface_yaw_deg": status.surface_yaw_deg,
            "using_depth": status.using_depth,
            "relative_size": status.relative_size,
        }


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
