import asyncio
from viam.module.module import Module

import mycobot_280_arm
import mycobot_gripper

if __name__ == "__main__":
    asyncio.run(Module.run_from_registry())
