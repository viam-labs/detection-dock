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
from viam.services.vision import VisionClient

import time
import asyncio

LOGGER = getLogger(__name__)

class detectionDock(Action, Reconfigurable):
    

    MODEL: ClassVar[Model] = Model(ModelFamily("viam-labs", "dock"), "detection-dock")
    
    power_sensor: PowerSensor
    base: Base
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
        detector = config.attributes.fields["detector"].string_value
        if detector == "":
            raise Exception("detector must be defined")
        detection_class = config.attributes.fields["detection_class"].string_value
        if detection_class == "":
            raise Exception("detection_class must be defined")
        return

    # Handles attribute reconfiguration
    def reconfigure(self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]):
        power_sensor = config.attributes.fields["power_sensor"].string_value
        actual_power_sensor = dependencies[PowerSensor.get_resource_name(power_sensor)]
        self.power_sensor = cast(PowerSensor, actual_power_sensor)

        base = config.attributes.fields["base"].string_value
        actual_base = dependencies[Base.get_resource_name(base)]
        self.base = cast(Base, actual_base)

        detector = config.attributes.fields["detector"].string_value
        actual_detector = dependencies[VisionClient.get_resource_name(detector)]
        self.detector = cast(VisionClient, actual_detector)

        self.detection_class = config.attributes.fields["detection_class"].string_value

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

    async def start(self) -> str:

        return "OK"
    
    async def stop(self) -> str:

        return "OK"

    async def is_running(self) -> bool:

        return True
    
    async def status(self) -> dict:

        return {}