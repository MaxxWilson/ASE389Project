import numpy as np

from pnc.state_machine import StateMachine
from pnc.draco_manipulation_pnc.draco_manipulation_state_provider import DracoManipulationStateProvider
from config.draco_manipulation_config import ManipulationConfig, LocomanipulationState
from util import util


class DoubleSupportHandReach(StateMachine):
    def __init__(self, id, tm, hm, fm, robot):
        super().__init__(id, robot)
        self._trajectory_managers = tm
        self._hierarchy_managers = hm
        self._force_managers = fm
        self._sp = DracoManipulationStateProvider()
        self._start_time = 0.
        self._moving_duration = 0.
        self._rh_target_pos = np.zeros(3)
        self._rh_target_quat = np.zeros(4)
        self._lh_target_pos = np.zeros(3)
        self._lh_target_quat = np.zeros(4)

    def one_step(self):
        self._state_machine_time = self._sp.curr_time - self._start_time

        # Update Foot Task
        self._trajectory_managers["lfoot"].use_current()
        self._trajectory_managers["rfoot"].use_current()

        # Update Hand Task
        if self._state_id == LocomanipulationState.RH_HANDREACH:
            self._trajectory_managers['rhand'].update_hand_trajectory(
                self._sp.curr_time)
        elif self._state_id == LocomanipulationState.LH_HANDREACH:
            self._trajectory_managers['lhand'].update_hand_trajectory(
                self._sp.curr_time)
        else:
            raise ValueError("Wrong LocomanipulationState: HandSide")

    def first_visit(self):
        self._start_time = self._sp.curr_time

        if self._state_id == LocomanipulationState.RH_HANDREACH:
            print("[LocomanipulationState] Right Hand Reaching")
            target_hand_iso = np.eye(4)
            target_hand_iso[0:3, 0:3] = util.quat_to_rot(self._rh_target_quat)
            target_hand_iso[0:3, 3] = self._rh_target_pos
            self._trajectory_managers['rhand'].initialize_hand_trajectory(
                self._start_time, self._moving_duration, target_hand_iso)
        elif self._state_id == LocomanipulationState.LH_HANDREACH:
            print("[LocomanipulationState] Left Hand Reaching")
            target_hand_iso = np.eye(4)
            target_hand_iso[0:3, 0:3] = util.quat_to_rot(self._lh_target_quat)
            target_hand_iso[0:3, 3] = self._lh_target_pos
            self._trajectory_managers['lhand'].initialize_hand_trajectory(
                self._start_time, self._moving_duration, target_hand_iso)
        else:
            raise ValueError("Wrong LocomanipulationState: HandSide")

    def last_visit(self):
        pass

    def end_of_state(self):
        pass

    def get_next_state(self):
        pass

    @property
    def moving_duration(self):
        return self._moving_duration

    @moving_duration.setter
    def moving_duration(self, value):
        self._moving_duration = value

    @property
    def rh_target_pos(self):
        return self._rh_target_pos

    @rh_target_pos.setter
    def rh_target_pos(self, value):
        self._rh_target_pos = value

    @property
    def rh_target_quat(self):
        return self._rh_target_quat

    @rh_target_quat.setter
    def rh_target_quat(self, value):
        self._rh_target_quat = value

    @property
    def lh_target_pos(self):
        return self._lh_target_pos

    @lh_target_pos.setter
    def lh_target_pos(self, value):
        self._lh_target_pos = value

    @property
    def lh_target_quat(self):
        return self._lh_target_quat

    @lh_target_quat.setter
    def lh_target_quat(self, value):
        self._lh_target_quat = value
