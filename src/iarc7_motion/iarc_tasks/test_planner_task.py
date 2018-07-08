#!/usr/bin/env python
import rospy
import threading
import time

from .abstract_task import AbstractTask

from iarc_tasks.task_states import (TaskRunning,
                                    TaskDone,
                                    TaskCanceled,
                                    TaskAborted)
from iarc_tasks.task_commands import NopCommand, GlobalPlanCommand

from iarc7_msgs.msg import PlanGoal, PlanAction, MotionPointStamped

class TestPlannerTaskState(object):
    init = 0
    waiting = 1
    done = 2


class TestPlannerTask(AbstractTask):

    def __init__(self, task_request):
        super(TestPlannerTask, self).__init__()

        self._lock = threading.RLock()

        self._planning_lag = rospy.Duration(.25)
        self._coordinate_frame_offset = 10

        self._plan = None

        self._linear_gen = self.topic_buffer.get_linear_motion_profile_generator()

        self._state = TestPlannerTaskState.init

    def get_desired_command(self):
        with self._lock:
            if self._state == TestPlannerTaskState.init:
                starting = self._linear_gen.expected_point_at_time(rospy.Time.now() + self._planning_lag)

                _pose = starting.motion_point.pose.position
                _vel = starting.motion_point.twist.linear

                request = PlanGoal()

                start = MotionPointStamped()
                start.motion_point.pose.position.x = _pose.x + self._coordinate_frame_offset
                start.motion_point.pose.position.y = _pose.y + self._coordinate_frame_offset
                start.motion_point.pose.position.z = _pose.z

                start.motion_point.twist.linear.x = _vel.x
                start.motion_point.twist.linear.y = _vel.y
                start.motion_point.twist.linear.z = _vel.z

                goal = MotionPointStamped()
                goal.motion_point.pose.position.x = 5
                goal.motion_point.pose.position.y = 5
                goal.motion_point.pose.position.z = 1

                request.start = start
                request.goal = goal

                x = time.time()

                result_ = self.topic_buffer.make_plan_request(request, self._feedback_callback)

                rospy.logerr('time to get result: ' + str(time.time()-x))

                self._plan = result_.plan

                self._linear_gen.set_global_plan(self._plan)

                self._state = TestPlannerTaskState.waiting

            if self._state == TestPlannerTaskState.waiting:
                self._state = TestPlannerTaskState.done
                return (TaskRunning(), GlobalPlanCommand(self._plan))

            return (TaskRunning(), NopCommand())

    def _feedback_callback(self, msg):
        with self._lock:
            self._feedback = msg
            rospy.logwarn('Feedback recieved')

    def cancel(self):
        rospy.loginfo("TestPlannerTask canceling")
        self.canceled = True
        return True

    def set_incoming_transition(self, transition):
        self._transition = transition
