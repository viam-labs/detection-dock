"""
This file registers the model with the Python SDK.
"""

from action_python import Action
from viam.resource.registry import Registry, ResourceCreatorRegistration

from .detectionDock import detectionDock

Registry.register_resource_creator(
    Action.API,
    detectionDock.MODEL,
    ResourceCreatorRegistration(detectionDock.new, detectionDock.validate),
)
