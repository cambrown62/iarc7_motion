#!/usr/bin/env python
import rospy

from .abstract_task import AbstractTask

from iarc_tasks.task_states import (TaskRunning,
                                    TaskDone,
                                    TaskCanceled,
                                    TaskAborted)
from iarc_tasks.task_commands import NopCommand

from iarc7_msgs.msg import PlanGoal, PlanAction, MotionPointStamped

class TestPlannerTask(AbstractTask):

    def __init__(self, task_request):
        super(TestPlannerTask, self).__init__()

        self._plan_canceled = False
        self._request_sent = False

    def get_desired_command(self):

        if not self._request_sent:
            goal = PlanGoal()
            goal.goal.motion_point.pose.position.x = 5
            goal.goal.motion_point.pose.position.y = 5
            goal.goal.motion_point.pose.position.z = 1

            self.topic_buffer.make_plan_request(goal, self._feedback_callback)
            self._request_sent = True

        if self._plan_canceled:
            return (TaskDone(), NopCommand())
        else:
            return (TaskRunning(),)

    def _feedback_callback(self, msg):
        self._feedback = msg
        rospy.logwarn('Feedback recieved')
        self.topic_buffer.cancel_plan_goal()
        self._plan_canceled = True

    def cancel(self):
        rospy.loginfo("TestPlannerTask canceling")
        self.canceled = True
        return True

    def set_incoming_transition(self, transition):
        self._transition = transition
