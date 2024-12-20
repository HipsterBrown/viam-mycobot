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

LOGGER = getLogger(__name__)


class GripperState(Enum):
    OPEN = 0
    CLOSED = 1


class GripperConfig(BaseModel):
    arm_name: str
    default_speed: int = Field(gt=0, le=100, default=50)


class MyCobot280(Gripper, EasyResource):
    MODEL: ClassVar[Model] = Model(ModelFamily("hipsterbrown", "gripper"), "mycobot")

    mycobot: Arm

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
        cfg = GripperConfig(**struct_to_dict(config.attributes))
        return [cfg.arm_name]

    def reconfigure(
        self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ):
        """This method allows you to dynamically update your service when it receives a new `config` object.

        Args:
            config (ComponentConfig): The new configuration
            dependencies (Mapping[ResourceName, ResourceBase]): Any dependencies (both implicit and explicit)
        """
        self.config = GripperConfig(**struct_to_dict(config.attributes))

        self.mycobot = cast(
            Arm, dependencies[Arm.get_resource_name(self.config.arm_name)]
        )

    async def open(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ):
        await self.mycobot.do_command(
            {
                "set_gripper_state": [
                    GripperState.OPEN.value,
                    int(self.config.default_speed),
                ]
            }
        )

    async def grab(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> bool:
        response = await self.mycobot.do_command(
            {
                "set_gripper_state": [
                    GripperState.CLOSED.value,
                    int(self.config.default_speed),
                ]
            }
        )
        return bool(response.get("set_gripper_state", False))

    async def stop(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ):
        pass

    async def is_moving(self) -> bool:
        response = await self.mycobot.do_command({"is_gripper_moving": []})
        return bool(response.get("is_gripper_moving", False))
