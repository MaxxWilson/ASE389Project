import numpy as np

from config.draco_manipulation_config import LocomanipulationState
from pnc.state_machine import StateMachine
from pnc.planner.locomotion.dcm_planner.footstep import Footstep
from pnc.draco_manipulation_pnc.draco_manipulation_state_provider import DracoManipulationStateProvider


class DoubleSupportBalance(StateMachine):
    def __init__(self, id, tm, hm, fm, robot):
        super(DoubleSupportBalance, self).__init__(id, robot)
        self._trajectory_managers = tm
        self._hierarchy_managers = hm
        self._force_managers = fm
        self._sp = DracoManipulationStateProvider()
        self._start_time = 0.
        self._walking_trigger = False
        self._swaying_trigger = False
        self._hand_task_trans_trigger = False

    @property
    def walking_trigger(self):
        return self._walking_trigger

    @walking_trigger.setter
    def walking_trigger(self, val):
        self._walking_trigger = val

    @property
    def swaying_trigger(self):
        return self._swaying_trigger

    @swaying_trigger.setter
    def swaying_trigger(self, value):
        self._swaying_trigger = value

    @property
    def hand_task_trans_trigger(self):
        return self._hand_task_trans_trigger

    @hand_task_trans_trigger.setter
    def hand_task_trans_trigger(self, value):
        self._hand_task_trans_trigger = value

    def one_step(self):
        self._state_machine_time = self._sp.curr_time - self._start_time

        # Update Foot Task
        self._trajectory_managers["lfoot"].use_current()
        self._trajectory_managers["rfoot"].use_current()

        # Update Hand Task
        # self._trajectory_managers["lhand"].use_current()
        # self._trajectory_managers["rhand"].use_current()

    def first_visit(self):
        print("[LocomanipulationState] BALANCE")
        self._walking_trigger = False
        self._start_time = self._sp.curr_time

    def last_visit(self):
        pass

    def end_of_state(self):
        if (self._walking_trigger) and (
                len(self._trajectory_managers["dcm"].footstep_list) > 0
        ) and not (self._trajectory_managers["dcm"].no_reaming_steps()):
            return True

        if self._hand_task_trans_trigger:
            self._b_hand_task_trans = True
            return True

        return False

    def get_next_state(self):
        b_valid_step, robot_side = self._trajectory_managers[
            "dcm"].next_step_side()
        if b_valid_step:
            if robot_side == Footstep.LEFT_SIDE:
                return LocomanipulationState.LF_CONTACT_TRANS_START
            elif robot_side == Footstep.RIGHT_SIDE:
                return LocomanipulationState.RF_CONTACT_TRANS_START
            else:
                raise ValueError("Wrong Footstep Side")

        if self._hand_task_trans_trigger:
            return LocomanipulationState.HT_TRANS
