from arca_sim.state import LowLevelState, LowLevelCommand, HighLevelCommand, HighLevelState
import numpy as np


class Controller:

    def __init__(self):
        ...

    def update(self, state: LowLevelState) -> LowLevelCommand:
        return LowLevelCommand(
            pos=np.zeros(6),
            vel=np.zeros(6),
            current=np.zeros(6),
            kp=np.zeros(6),
            kd=np.zeros(6)
        )

    def get_low_level_command(self, state: HighLevelState) -> LowLevelCommand:
        ...

    def get_high_level_state(self, state: LowLevelState) -> HighLevelState:
        ...
