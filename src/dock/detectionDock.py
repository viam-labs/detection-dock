import asyncio
import math
import time
from dataclasses import dataclass
from typing import Any, ClassVar, Mapping, Optional, Sequence, Tuple, cast

from typing_extensions import Self
from viam.components.base import Base
from viam.components.camera import Camera
from viam.components.power_sensor import PowerSensor
from viam.logging import getLogger
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


@dataclass
class Status:
    is_running: bool = False
    is_docked: bool = False
    state: str = "idle"
    retry_count: int = 0
    bearing_deg: float = 0.0
    relative_size: float = 0.0


class detectionDock(Action, Reconfigurable):
    MODEL: ClassVar[Model] = Model(ModelFamily("viam-labs", "dock"), "detection-dock")

    power_sensor: Optional[PowerSensor]
    base: Base
    camera: Camera
    detector: VisionClient
    internal_status: Status

    @classmethod
    def new(cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]) -> Self:
        my_class = cls(config.name)
        my_class.internal_status = Status()
        my_class.power_sensor = None
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
        power_sensor = fields["power_sensor"].string_value if "power_sensor" in fields else ""
        optional = [power_sensor] if power_sensor else []
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

        self.detection_class = _string(fields, "detection_class", "match")
        self.camera_fov_deg = _number(fields, "camera_fov_deg", 70.0)
        self.close_percent = _number(fields, "close_percent", 0.45)
        self.center_tolerance = _number(fields, "center_tolerance", 0.05)
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

        self.controller_frequency = _number(fields, "controller_frequency", 8.0)
        self.initial_perception_timeout = _number(fields, "initial_perception_timeout", 15.0)
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
        deadline = time.monotonic() + self.initial_perception_timeout
        while self.internal_status.is_running and time.monotonic() < deadline:
            sample = await self._detect()
            if sample is not None:
                await self.base.stop()
                return True
            await self._command(0.0, self.v_angular_max)
            await asyncio.sleep(1.0 / self.controller_frequency)
        await self.base.stop()
        return False

    async def _approach(self) -> bool:
        self.internal_status.state = "approaching"
        deadline = time.monotonic() + self.dock_approach_timeout
        last_seen = time.monotonic()
        filtered_center: Optional[float] = None
        filtered_size: Optional[float] = None

        while self.internal_status.is_running and time.monotonic() < deadline:
            loop_start = time.monotonic()
            sample = await self._detect()
            if sample is None:
                if time.monotonic() - last_seen > self.external_detection_timeout:
                    LOGGER.info("lost dock detection")
                    await self.base.stop()
                    return False
            else:
                last_seen = time.monotonic()
                center_offset, relative_size = sample
                if filtered_center is None or filtered_size is None:
                    filtered_center, filtered_size = center_offset, relative_size
                else:
                    coef = self.filter_coef
                    filtered_center = (1.0 - coef) * filtered_center + coef * center_offset
                    filtered_size = (1.0 - coef) * filtered_size + coef * relative_size

                psi = -filtered_center * math.radians(self.camera_fov_deg)
                distance = self.docking_distance * (self.close_percent / max(filtered_size, 1e-3))
                self.internal_status.bearing_deg = math.degrees(psi)
                self.internal_status.relative_size = filtered_size

                if filtered_size >= self.close_percent and abs(filtered_center) <= self.center_tolerance:
                    await self.base.stop()
                    return True

                linear, angular = self._approach_velocity(psi, distance)
                await self._command(linear, angular)

            elapsed = time.monotonic() - loop_start
            await asyncio.sleep(max(0.0, (1.0 / self.controller_frequency) - elapsed))

        await self.base.stop()
        return False

    def _approach_velocity(self, psi: float, distance: float) -> Tuple[float, float]:
        goal_dist = max(distance - self.docking_distance, 1e-3)
        r, phi, delta = ego_polar(goal_dist * math.cos(psi), goal_dist * math.sin(psi), psi)
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

    async def _detect(self) -> Optional[Tuple[float, float]]:
        images, _ = await self.camera.get_images()
        if not images:
            return None
        image = images[0]
        width = image.width or 0
        if width <= 0:
            return None

        detections = await self.detector.get_detections(image)
        matches = [det for det in detections if not self.detection_class or det.class_name == self.detection_class]
        if not matches:
            return None

        best = max(matches, key=lambda det: (det.x_max - det.x_min) * (det.y_max - det.y_min))
        center_offset = ((best.x_min + best.x_max) / 2.0) / width - 0.5
        relative_size = (best.x_max - best.x_min) / width
        return center_offset, relative_size

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
