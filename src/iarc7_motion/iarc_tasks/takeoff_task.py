#!/usr/bin/env python

import rospy

from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import TwistStamped

from iarc7_msgs.msg import FlightControllerStatus

from .abstract_task import AbstractTask
from iarc_tasks.task_states import (TaskRunning,
                                    TaskDone,
                                    TaskCanceled,
                                    TaskAborted,
                                    TaskFailed)
from iarc_tasks.task_commands import (ArmCommand,
                                      NopCommand,
                                      GroundInteractionCommand)

class TakeoffTaskState:
    init = 0
    request_arm = 1
    pause_before_takeoff = 2
    takeoff = 3
    done = 4
    failed = 5

class TakeoffTask(AbstractTask):

    def __init__(self, actionvalues_dict):
        self._canceled = False;

        try:
            self._DELAY_BEFORE_TAKEOFF = rospy.get_param('~delay_before_takeoff')
        except KeyError as e:
            rospy.logerr('Could not lookup a parameter for takeoff task')
            raise

        self._fc_status = None
        self._fc_status_sub = rospy.Subscriber('fc_status', FlightControllerStatus, self._receive_fc_status)
        self._state = TakeoffTaskState.init
        self._arm_request_success = False

    def _receive_fc_status(self, data):
        self._fc_status = data

    def arm_callback(self, data):
        self._arm_request_success = data.success

    def takeoff_callback(self, status, result):
        if status == GoalStatus.SUCCEEDED:
            # Takeoff request succeeded, transition state
            self._state = TakeoffTaskState.done
        else:
            # Takeoff request failed
            rospy.logerr('Takeoff task failed during call to low level motion')
            self._state = TakeoffTaskState.failed

    def get_desired_command(self):
        if self._canceled:
            return (TaskCanceled(),)

        if self._state == TakeoffTaskState.init:
            # Check if we have an fc status
            if self._fc_status is None:
                return (TaskRunning(), NopCommand())
            # Check that auto pilot is enabled
            if not self._fc_status.auto_pilot:
                return (TaskFailed(msg='flight controller not allowing auto pilot'),)
            # Check that the FC is not already armed
            if self._fc_status.armed:
                return (TaskFailed(msg='flight controller armed prior to takeoff'),)

            # All is good change state to request arm
            self._state = TakeoffTaskState.request_arm

        # Enter the arming request stage
        if self._state == TakeoffTaskState.request_arm:
            # Check that the FC is not already armed
            if self._arm_request_success:
                self.pause_start_time = rospy.Time.now()
                self._state = TakeoffTaskState.pause_before_takeoff
            else:
                return (TaskRunning(), ArmCommand(True, self.arm_callback))

        # Pause before ramping up the motors
        if self._state == TakeoffTaskState.pause_before_takeoff:
            if rospy.Time.now() - self.pause_start_time > rospy.Duration(self._DELAY_BEFORE_TAKEOFF):
                self._state = TakeoffTaskState.takeoff
                return (TaskRunning(), GroundInteractionCommand(
                                       'takeoff',
                                       self.takeoff_callback))
            else:
                return (TaskRunning(), NopCommand())

        # Enter the takeoff phase
        if self._state == TakeoffTaskState.takeoff:
            return (TaskRunning(),)

        if self._state == TakeoffTaskState.done:
            return (TaskDone(), NopCommand())

        if self._state == TakeoffTaskState.failed:
            return (TaskFailed(msg='Take off task experienced failure'),)

        # Impossible state reached
        return (TaskAborted(msg='Impossible state in takeoff task reached'),)


    def cancel(self):
        rospy.loginfo('TakeoffTask canceled')
        self._canceled = True
