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
    feedback_recieved = 1
    waiting = 2
    done = 3

class TestPlannerTask(AbstractTask):
    def __init__(self, task_request):
        super(TestPlannerTask, self).__init__()

        self._lock = threading.RLock()

        self._linear_gen = self.topic_buffer.get_linear_motion_profile_generator()

        self._planning_lag = rospy.Duration(.30)
        self._coordinate_frame_offset = 10.0

        self._plan = None

        self._state = TestPlannerTaskState.init

    def get_desired_command(self):
        with self._lock:
            expected_time = rospy.Time.now() + self._planning_lag
            starting = self._linear_gen.expected_point_at_time(expected_time)

            _pose = starting.motion_point.pose.position
            _vel = starting.motion_point.twist.linear

            corrected_x = _pose.x + self._coordinate_frame_offset
            corrected_y = _pose.y + self._coordinate_frame_offset
            corrected_z = _pose.z

            done = abs(corrected_x-15) < .2 and abs(corrected_y-15) < .2 and abs(corrected_z-1) < .1

            if done:
                self.topic_buffer.cancel_plan_goal()
                return (TaskDone(),)

            if self._state == TestPlannerTaskState.init:
                request = PlanGoal()
                request.header.stamp = expected_time

                start = MotionPointStamped()
                start.motion_point.pose.position.x = corrected_x
                start.motion_point.pose.position.y = corrected_y
                start.motion_point.pose.position.z = corrected_z

                start.motion_point.twist.linear.x = _vel.x
                start.motion_point.twist.linear.y = _vel.y
                start.motion_point.twist.linear.z = _vel.z

                goal = MotionPointStamped()
                goal.motion_point.pose.position.x = 5 + self._coordinate_frame_offset
                goal.motion_point.pose.position.y = 5 + self._coordinate_frame_offset
                goal.motion_point.pose.position.z = 1

                request.start = start
                request.goal = goal

                self.topic_buffer.make_plan_request(request, self._feedback_callback)

                self._state = TestPlannerTaskState.waiting

            if self._state == TestPlannerTaskState.feedback_recieved:
                # make a new plan request since we have received feedback
                self._state = TestPlannerTaskState.waiting

                request = PlanGoal()
                request.header.stamp = expected_time

                start = MotionPointStamped()
                start.motion_point.pose.position.x = _pose.x + self._coordinate_frame_offset
                start.motion_point.pose.position.y = _pose.y + self._coordinate_frame_offset
                start.motion_point.pose.position.z = _pose.z

                start.motion_point.twist.linear.x = _vel.x
                start.motion_point.twist.linear.y = _vel.y
                start.motion_point.twist.linear.z = _vel.z

                goal = MotionPointStamped()
                goal.motion_point.pose.position.x = 5 + self._coordinate_frame_offset
                goal.motion_point.pose.position.y = 5 + self._coordinate_frame_offset
                goal.motion_point.pose.position.z = 1

                request.start = start
                request.goal = goal

                self.topic_buffer.make_plan_request(request, self._feedback_callback)

                if self._feedback.success:
                    # send LLM the plan we recieved
                    self._linear_gen.set_global_plan(self._plan)
                    return (TaskRunning(), GlobalPlanCommand(self._plan))
                else:
                    # planning failed, nop
                    return (TaskRunning(), NopCommand())

            return (TaskRunning(), NopCommand())

    def _feedback_callback(self, msg):
        with self._lock:
            self._feedback = msg
            self._plan = msg.plan
            self._state = TestPlannerTaskState.feedback_recieved
            rospy.logwarn('Feedback recieved')

    def cancel(self):
        rospy.loginfo("TestPlannerTask canceling")
        self.canceled = True
        return True

    def set_incoming_transition(self, transition):
        self._transition = transition
