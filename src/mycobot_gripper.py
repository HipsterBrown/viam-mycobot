from enum import Enum
from typing import Any, ClassVar, Dict, Mapping, Optional, Sequence, cast

from typing_extensions import Self
from viam.components.arm import Arm
from viam.components.gripper import Gripper
from viam.logging import getLogger
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import ResourceName
from viam.resource.base import ResourceBase
from viam.resource.easy_resource import EasyResource
from viam.resource.types import Model, ModelFamily
from viam.utils import struct_to_dict

from pydantic import BaseModel, Field

from controller import MyCobotController

LOGGER = getLogger(__name__)


class GripperState(Enum):
    OPEN = 0
    CLOSED = 1


class GripperConfig(BaseModel):
    default_speed: int = Field(gt=0, le=100, default=50)


class MyCobot280(Gripper, EasyResource):
    MODEL: ClassVar[Model] = Model(ModelFamily("hipsterbrown", "gripper"), "mycobot")

    mycobot: MyCobotController

    @classmethod
    def new(
        cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ) -> Self:
        """This method creates a new instance of this Gripper component.
        The default implementation sets the name from the `config` parameter and then calls `reconfigure`.

        Args:
            config (ComponentConfig): The configuration for this resource
            dependencies (Mapping[ResourceName, ResourceBase]): The dependencies (both implicit and explicit)

        Returns:
            Self: The resource
        """
        return super().new(config, dependencies)

    @classmethod
    def validate_config(cls, config: ComponentConfig) -> Sequence[str]:
        """This method allows you to validate the configuration object received from the machine,
        as well as to return any implicit dependencies based on that `config`.

        Args:
            config (ComponentConfig): The configuration for this resource

        Returns:
            Sequence[str]: A list of implicit dependencies
        """
        GripperConfig(**struct_to_dict(config.attributes))
        return []

    def reconfigure(
        self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ):
        """This method allows you to dynamically update your service when it receives a new `config` object.

        Args:
            config (ComponentConfig): The new configuration
            dependencies (Mapping[ResourceName, ResourceBase]): Any dependencies (both implicit and explicit)
        """
        self.config = GripperConfig(**struct_to_dict(config.attributes))
        self.mycobot = MyCobotController()

    async def open(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ):
        self.mycobot.client.set_gripper_state(
            GripperState.OPEN.value, int(self.config.default_speed)
        )

    async def grab(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> bool:
        self.mycobot.client.set_gripper_state(
            GripperState.CLOSED.value, int(self.config.default_speed)
        )
        return True

    async def stop(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ):
        self.mycobot.client.stop()

    async def is_moving(self) -> bool:
        is_moving = self.mycobot.client.is_moving()
        return is_moving == 1
