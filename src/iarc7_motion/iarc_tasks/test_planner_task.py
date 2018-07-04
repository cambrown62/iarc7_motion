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
            if self.topic_buffer.has_odometry_message():
                curr_odom = self.topic_buffer.get_odometry_message()
                _pose = curr_odom.pose.pose.point
                _vel = curr_odom.twist.twist.linear

                request = PlanGoal()

                start = MotionPointStamped()
                start.motion_point.pose.position.x = _pose.x
                start.motion_point.pose.position.y = _pose.y
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

                self.topic_buffer.make_plan_request(request, self._feedback_callback)
                self._request_sent = True
            else:
                rospy.logerr('No odom available to make plan request.')

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
