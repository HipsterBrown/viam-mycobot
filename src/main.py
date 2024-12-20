import asyncio
from typing import Any, ClassVar, Dict, Mapping, Optional, Sequence, Tuple

from typing_extensions import Self
from viam.components.arm import Arm, Pose, JointPositions, KinematicsFileFormat
from viam.logging import getLogger
from viam.module.module import Module
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import ResourceName
from viam.resource.base import ResourceBase
from viam.resource.easy_resource import EasyResource
from viam.resource.types import Model, ModelFamily

from pymycobot.mycobot280 import MyCobot280 as _MyCobot
from pymycobot import PI_PORT, PI_BAUD

LOGGER = getLogger(__name__)


class MyCobot280(Arm, EasyResource):
    MODEL: ClassVar[Model] = Model(ModelFamily("hipsterbrown", "arm"), "mycobot280")

    mycobot: Optional[_MyCobot] = None

    @classmethod
    def new(
        cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ) -> Self:
        """This method creates a new instance of this Arm component.
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
        return []

    def reconfigure(
        self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ):
        """This method allows you to dynamically update your service when it receives a new `config` object.

        Args:
            config (ComponentConfig): The new configuration
            dependencies (Mapping[ResourceName, ResourceBase]): Any dependencies (both implicit and explicit)
        """
        if self.mycobot is not None:
            LOGGER.info("Resetting connection")
            self.mycobot.stop()
            self.mycobot.close()

        self.mycobot = _MyCobot(PI_PORT, PI_BAUD)
        self.mycobot.set_color(0, 0, 255)

    async def get_end_position(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> Pose:
        if not self.mycobot:
            return Pose(x=0.0, y=0.0, z=0.0, o_x=0.0, o_y=0.0, o_z=0.0)

        x, y, z, rx, ry, rz = self.mycobot.get_coords()
        return Pose(x=x, y=y, z=z, o_x=rx, o_y=ry, o_z=rz)

    async def move_to_position(
        self,
        pose: Pose,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ):
        LOGGER.info(
            f"Moving to Pose: x = {pose.x}, y = {pose.y}, z = {pose.z}, o_x = {pose.o_x}, o_y = {pose.o_y}, o_z = {pose.o_z}"
        )
        if not self.mycobot:
            return

        self.mycobot.send_coords(
            [pose.x, pose.y, pose.z, pose.o_x, pose.o_y, pose.o_z], 20, 1
        )

    async def move_to_joint_positions(
        self,
        positions: JointPositions,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ):
        LOGGER.info(f"Moving to positions: {list(positions.values)}")
        if not self.mycobot:
            LOGGER.warning("mycobot not available")
            return

        angles = list(positions.values)
        self.mycobot.send_angles(angles, 20)

    async def get_joint_positions(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> JointPositions:
        if not self.mycobot:
            return JointPositions(values=[])
        angles = self.mycobot.get_angles()
        return JointPositions(values=angles)

    async def stop(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ):
        if not self.mycobot:
            return

        self.mycobot.stop()

    async def is_moving(self) -> bool:
        if not self.mycobot:
            return False

        return self.mycobot.is_moving() == 1

    async def get_kinematics(
        self, *, extra: Optional[Dict[str, Any]] = None, timeout: Optional[float] = None
    ) -> Tuple[KinematicsFileFormat.ValueType, bytes]:
        with open("assets/mycobot_280_pi.urdf", "rb") as f:
            data = f.read()

        return (KinematicsFileFormat.KINEMATICS_FILE_FORMAT_URDF, data)

    async def close(self):
        if not self.mycobot:
            return

        self.mycobot.stop()
        self.mycobot.close()
        self.mycobot = None


if __name__ == "__main__":
    asyncio.run(Module.run_from_registry())
