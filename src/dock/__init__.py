"""
This file registers the model with the Python SDK.
"""

from viam.resource.registry import Registry, ResourceCreatorRegistration

from action_python import Action
from .detectionDock import detectionDock

Registry.register_resource_creator(Action.SUBTYPE, detectionDock.MODEL, ResourceCreatorRegistration(detectionDock.new))
