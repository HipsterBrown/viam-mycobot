from threading import Lock
from typing_extensions import Self
import weakref
from pymycobot.mycobot280 import MyCobot280 as _MyCobot

# from pymycobot import PI_PORT, PI_BAUD
PI_PORT = "/dev/ttyAMA0"
PI_BAUD = 1000000


class MyCobotController:
    _instance = None
    _lock = Lock()
    _ref_count = 0
    _refs = set()
    client: _MyCobot

    def __new__(cls) -> Self:
        with cls._lock:
            if cls._instance is None:
                cls._instance = super(MyCobotController, cls).__new__(cls)
                cls._instance._initialized = False
        return cls._instance

    def __init__(self) -> None:
        with self._lock:
            if not self._initialized:
                self._initialized = True
                self.client = _MyCobot(PI_PORT, PI_BAUD)

            type(self)._ref_count += 1
            ref = weakref.ref(self, self._cleanup)
            type(self)._refs.add(ref)

    @classmethod
    def _cleanup(cls, ref):
        with cls._lock:
            if ref in cls._refs:
                cls._refs.remove(ref)
                cls._ref_count -= 1

                if cls._ref_count == 0 and cls._instance:
                    cls._instance.client.stop()
                    cls._instance.client.close()
                    cls._instance = None

    def __del__(self):
        with type(self)._lock:
            type(self)._ref_count -= 1
            if type(self)._ref_count <= 0:
                self.client.stop()
                self.client.close()
