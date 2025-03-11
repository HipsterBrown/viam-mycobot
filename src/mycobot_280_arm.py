import math
from typing import Any, ClassVar, Dict, Mapping, Optional, Sequence, Tuple

from typing_extensions import Self
from viam.components.arm import Arm, Pose, JointPositions, KinematicsFileFormat
from viam.logging import getLogger
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import ResourceName
from viam.resource.base import ResourceBase
from viam.components.component_base import ValueTypes
from viam.resource.easy_resource import EasyResource
from viam.resource.types import Model, ModelFamily
from viam.utils import struct_to_dict

from pymycobot.mycobot280 import MyCobot280 as _MyCobot
from pymycobot import PI_PORT, PI_BAUD

from pydantic import BaseModel, Field

from scipy.spatial.transform import Rotation

from controller import MyCobotController

LOGGER = getLogger("myCobot")


class ArmConfig(BaseModel):
    default_speed: int = Field(gt=0, le=100, default=20)


class MyCobot280(Arm, EasyResource):
    MODEL: ClassVar[Model] = Model(ModelFamily("hipsterbrown", "arm"), "mycobot280")

    mycobot: MyCobotController

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
        ArmConfig(**struct_to_dict(config.attributes))
        return []

    def reconfigure(
        self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ):
        """This method allows you to dynamically update your service when it receives a new `config` object.

        Args:
            config (ComponentConfig): The new configuration
            dependencies (Mapping[ResourceName, ResourceBase]): Any dependencies (both implicit and explicit)
        """
        self.config = ArmConfig(**struct_to_dict(config.attributes))

        self.mycobot = MyCobotController()
        self.mycobot.client.set_color(0, 0, 255)

        LOGGER.info(
            f"Current system version: {self.mycobot.client.get_system_version()}"
        )
        LOGGER.info(f"Current basic version: {self.mycobot.client.get_basic_version()}")
        LOGGER.info(f"Current reboot count: {self.mycobot.client.get_reboot_count()}")
        LOGGER.info(
            f"Current tool reference: {self.mycobot.client.get_tool_reference()}"
        )
        LOGGER.info(
            f"Current world reference: {self.mycobot.client.get_world_reference()}"
        )

    async def get_end_position(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> Pose:
        if not self.mycobot:
            return Pose(x=0.0, y=0.0, z=0.0, o_x=0.0, o_y=0.0, o_z=0.0)
        coords = self.mycobot.client.get_coords()
        LOGGER.info(coords)
        x, y, z, rx, ry, rz = coords
        rotation = Rotation.from_euler("xyz", [rx, ry, rz], degrees=True)
        o_x, o_y, o_z = rotation.as_rotvec()
        return Pose(x=x, y=y, z=z, o_x=o_x, o_y=o_y, o_z=o_z)

    async def move_to_position(
        self,
        pose: Pose,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ):
        if not self.mycobot:
            return

        current_coords = self.mycobot.client.get_coords()
        _x, _y, _z, rx, ry, rz = current_coords
        LOGGER.info(f"Pose Vector: o_x: {pose.o_x}, o_y: {pose.o_y}, o_z: {pose.o_z}")
        # rx, ry, rz = self._vector_to_angles(pose.o_x, pose.o_y, pose.o_z, pose.theta)
        rotation = Rotation.from_rotvec([pose.o_x, pose.o_y, pose.o_z])
        new_rx, new_ry, new_rz = rotation.as_euler("xyz", degrees=True)
        LOGGER.info(
            f"Rotational angles: rx: {rx} == {new_rx}, ry: {ry} == {new_ry}, rz: {rz} == {new_rz}"
        )
        LOGGER.info(
            f"Spatial coords: {_x} == {pose.x}, {_y} == {pose.y}, {_z} == {pose.z}"
        )

        self.mycobot.client.send_coords(
            [
                _x,
                _y,
                _z + 25,
                rx,
                ry,
                rz,
            ],
            self.config.default_speed,
            0,
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

        # possible ranges
        # joint 1: -168 <=> 168
        # joint 2: -135 <=> 135
        # joint 3: -150 <=> 150
        # joint 4: -145 <=> 145
        # joint 5: -165 <=> 165
        # joint 6: -180 <=> 180

        angles = list(positions.values)
        self.mycobot.client.send_angles(angles, self.config.default_speed)

    async def get_joint_positions(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> JointPositions:
        if not self.mycobot:
            return JointPositions(values=[])
        angles = self.mycobot.client.get_angles()
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

        self.mycobot.client.stop()

    async def is_moving(self) -> bool:
        if not self.mycobot:
            return False

        return self.mycobot.client.is_moving() == 1

    async def get_kinematics(
        self, *, extra: Optional[Dict[str, Any]] = None, timeout: Optional[float] = None
    ) -> Tuple[KinematicsFileFormat.ValueType, bytes]:
        LOGGER.info("Getting arm kinematics file")
        with open("assets/mycobot_280_pi.urdf", "rb") as f:
            data = f.read()

        return (KinematicsFileFormat.KINEMATICS_FILE_FORMAT_URDF, data)

    async def do_command(
        self,
        command: Mapping[str, ValueTypes],
        *,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> Mapping[str, ValueTypes]:
        if not self.mycobot:
            return {}

        LOGGER.info(f"do_command: {command}")
        result = {}
        for name, args in command.items():
            if name == "is_gripper_moving":
                result["is_gripper_moving"] = (
                    self.mycobot.client.is_gripper_moving() == 1
                )
            if name == "set_gripper_state":
                state, speed = list(args)
                result["set_gripper_state"] = (
                    self.mycobot.client.set_gripper_state(int(state), int(speed)) == 1
                )
        return result

    async def close(self):
        if not self.mycobot:
            return

        del self.mycobot

    def _angles_to_vector(self, ry: float, rz: float) -> Tuple[float, float, float]:
        o_x = math.cos(rz) * math.cos(ry)
        o_y = math.sin(ry)
        o_z = math.sin(rz) * math.cos(ry)

        length = math.sqrt(o_x * o_x + o_y * o_y + o_z * o_z)

        return (o_x / length, o_y / length, o_z / length)

    def _vector_to_angles(
        self, o_x: float, o_y: float, o_z: float, theta: float
    ) -> Tuple[float, float, float]:
        rx = theta  # rotation x information is lost in orientation vector
        ry = math.asin(o_y)
        rz = math.atan2(o_z, o_x)

        if abs(abs(ry) - math.pi / 2) < 0.001:
            rx = 0
            if ry > 0:
                rz += theta
            else:
                rz -= theta

        return (rx, ry, rz)

    def _degrees_to_radians(self, degrees: float) -> float:
        return degrees * (math.pi / 180)

    def _radians_to_degrees(self, radians: float) -> float:
        return radians * (180 / math.pi)
