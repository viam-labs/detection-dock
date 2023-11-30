from typing import ClassVar, Mapping, Sequence, Any, Dict, Optional, Tuple, Final, List, cast
from typing_extensions import Self

from viam.module.types import Reconfigurable
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import ResourceName, Vector3
from viam.resource.base import ResourceBase
from viam.resource.types import Model, ModelFamily

from action_python import Action
from viam.logging import getLogger

from viam.components.power_sensor import PowerSensor
from viam.components.base import Base
from viam.components.camera import Camera
from viam.services.vision import VisionClient

import time
import asyncio

LOGGER = getLogger(__name__)

class Status():
    is_running: bool
    is_docked: bool
    dock_try_count: int
    search_try_count: int
    detection_try_count: int

class detectionDock(Action, Reconfigurable):
    

    MODEL: ClassVar[Model] = Model(ModelFamily("viam-labs", "dock"), "detection-dock")
    
    power_sensor: PowerSensor
    base: Base
    camera: Camera
    detector: VisionClient
    detection_class: str
    spin_velocity: int
    straight_velocity: int
    search_spin_deg: int
    straight_distance: int
    center_tolerance: float
    detection_try_max: int
    close_percent: float
    max_search_tries: int
    max_dock_tries: int
    internal_status: Status

    # Constructor
    @classmethod
    def new(cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]) -> Self:
        my_class = cls(config.name)
        my_class.reconfigure(config, dependencies)
        return my_class

    # Validates JSON Configuration
    @classmethod
    def validate(cls, config: ComponentConfig):
        power_sensor = config.attributes.fields["power_sensor"].string_value
        if power_sensor == "":
            raise Exception("power_sensor must be defined")
        base = config.attributes.fields["base"].string_value
        if base == "":
            raise Exception("base must be defined")
        camera = config.attributes.fields["camera"].string_value
        if camera == "":
            raise Exception("camera must be defined")
        detector = config.attributes.fields["detector"].string_value
        if detector == "":
            raise Exception("detector must be defined")
        return

    # Handles attribute reconfiguration
    def reconfigure(self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]):
        power_sensor = config.attributes.fields["power_sensor"].string_value
        actual_power_sensor = dependencies[PowerSensor.get_resource_name(power_sensor)]
        self.power_sensor = cast(PowerSensor, actual_power_sensor)

        base = config.attributes.fields["base"].string_value
        actual_base = dependencies[Base.get_resource_name(base)]
        self.base = cast(Base, actual_base)

        camera = config.attributes.fields["camera"].string_value
        actual_camera = dependencies[Camera.get_resource_name(camera)]
        self.camera = cast(Camera, actual_camera)

        detector = config.attributes.fields["detector"].string_value
        actual_detector = dependencies[VisionClient.get_resource_name(detector)]
        self.detector = cast(VisionClient, actual_detector)

        self.detection_class = config.attributes.fields["detection_class"].string_value or "match"

        self.spin_velocity = int(config.attributes.fields["spin_velocity"].number_value or 800)
        self.straight_velocity = int(config.attributes.fields["straight_velocity"].number_value or 350)
        self.search_spin_deg = int(config.attributes.fields["search_spin_deg"].number_value or 4)
        self.straight_distance = int(config.attributes.fields["straight_distance"].number_value or 50)
        self.center_tolerance = float(config.attributes.fields["center_tolerance"].number_value or .05)
        self.detection_try_max = int(config.attributes.fields["detection_try_max"].number_value or 4)
        self.close_percent = float(config.attributes.fields["close_percent"].number_value or .45)
        self.max_search_tries = int(config.attributes.fields["max_search_tries"].number_value or 100)
        self.max_dock_tries = int(config.attributes.fields["max_dock_tries"].number_value or 10)

        return

    async def dock(self):
        self.internal_status.is_running = True
        self.internal_status.is_docked = False
        self.internal_status.search_try_count = 0
        self.internal_status.dock_try_count = 0
        self.internal_status.detection_try_count = 0

        while (not self.internal_status.is_docked) and self.internal_status.is_running and (self.internal_status.search_try_count < self.max_search_tries) and (self.internal_status.dock_try_count < self.max_dock_tries):
            img = await self.camera.get_image()
            detections = await self.detector.get_detections(img)

            if len(detections) == 1:
                print(detections)
                self.internal_status.search_try_count = 0

                relative_size = (detections[0].x_max - detections[0].x_min)/img.width
                centered = (detections[0].x_min + ((detections[0].x_max - detections[0].x_min)/2)) /img.width - .5

                print(centered, relative_size)
        
                # try to get it more centered
                if abs(centered) > self.center_tolerance:
                    to_spin = (abs(centered) - self.center_tolerance)/.04
                    if centered > 0:
                        print("centering right " + str(to_spin))
                        await self.base.spin(to_spin, -self.spin_velocity)
                    else:
                        print("centering left " + str(to_spin))
                        await self.base.spin(to_spin, self.spin_velocity)
                else:
                    print("moving forward")
                    await self.base.move_straight(self.straight_distance,self.straight_velocity)
                    if relative_size > self.close_percent:
                        self.internal_status.dock_try_count = self.internal_status.detection_try_count + 1
                        docked = await self.final_dock_routine()
                        if docked:
                            self.internal_status.is_docked = True
                        else:
                            # perform a backwards move to try again
                            await self.base.move_straight(-self.straight_distance*10, self.straight_velocity)
                    time.sleep(.1)
            else:
                self.internal_status.detection_try_count = self.internal_status.detection_try_count + 1
                if self.internal_status.detection_try_count > self.detection_try_max:
                    print("searching")
                    self.internal_status.search_try_count = self.internal_status.search_try_count + 1
                    await self.base.spin(self.search_spin_deg, self.spin_velocity)
                    self.internal_status.detection_try_count = 0
    
        self.internal_status.is_running = False

    async def final_dock_routine(self):

        current_voltage = await self.power_sensor.get_voltage()

        # finish by one big forward movement then a wiggle to make sure attached to dock
        await self.base.move_straight(self.straight_distance*5,self.straight_velocity*2)
        await self.base.spin(self.search_spin_deg*2, self.straight_velocity)
        await self.base.spin(self.search_spin_deg*2, -self.straight_velocity)
        await self.base.spin(self.search_spin_deg*2, self.straight_velocity)
        await self.base.spin(self.search_spin_deg*2, -self.straight_velocity)
        await self.base.move_straight(self.straight_distance,self.straight_velocity*2)
        await self.base.move_straight(int(self.straight_distance*.3),-self.straight_velocity*2)
    
        new_voltage = await self.power_sensor.get_voltage()

        # voltage should jump up when successfully docked
        if (new_voltage - current_voltage) > .12:
            return True
        else:
            return False
    
    async def start(self) -> str:
        asyncio.ensure_future(self.dock())
        return "OK"
    
    async def stop(self) -> str:
        self.internal_status.is_running = False
        return "OK"

    async def is_running(self) -> bool:
        return self.internal_status.is_running
    
    async def status(self) -> dict:
        return self.internal_status.__dict__