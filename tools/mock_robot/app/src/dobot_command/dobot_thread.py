"""dobot thread."""
# Copyright 2022 HarvestX Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import logging
import threading
import time

from utilities.function_parser import FunctionParser

from .dobot_hardware import DobotHardware
from .motion_command import MotionCommands


class DobotThread(threading.Thread):
    """DobotThread"""
    logger: logging.Logger

    def __init__(self, dobot: DobotHardware) -> None:
        super(DobotThread, self).__init__()
        self.logger = logging.getLogger("Dobot_Thread")
        self.__dobot = dobot
        self.__motion_commands = MotionCommands(dobot)
        self.__timestep = self.__dobot.get_timestep()

    def run(self):
        """run — precision timing loop using perf_counter dt integration."""
        TIMESTEP = self.__timestep  # nominal 5 ms
        last_t = time.perf_counter()

        while True:
            t0 = time.perf_counter()

            # Process one motion command if robot is ready
            empty, command = self.__dobot.motion_unstack()
            if not empty:
                self.logger.info(command)
                try:
                    FunctionParser.exec(self.__motion_commands, command)
                except ValueError as err:
                    self.logger.error(err)

            # Compute *actual* elapsed time since last physics step
            now = time.perf_counter()
            dt = now - last_t
            last_t = now

            # Clamp dt: ignore spikes >3× nominal (e.g. system suspend)
            dt = min(dt, TIMESTEP * 3)

            # Update physics with the real dt
            self.__dobot.update_status(dt=dt)

            # Sleep only the remaining budget to hit the next 5 ms tick
            elapsed = time.perf_counter() - t0
            sleep_time = TIMESTEP - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
