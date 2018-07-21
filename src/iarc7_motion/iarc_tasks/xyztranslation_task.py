import math
import rospy
import tf2_ros
import threading

from .abstract_task import AbstractTask
from iarc_tasks.task_states import (TaskRunning,
                                    TaskDone,
                                    TaskCanceled,
                                    TaskAborted,
                                    TaskFailed)
from iarc_tasks.task_commands import NopCommand, GlobalPlanCommand

from iarc7_msgs.msg import PlanGoal, PlanAction, MotionPointStamped

class XYZTranslationTaskState(object):
    INIT = 1
    WAITING = 2
    PLAN_RECEIVED = 3
    COMPLETING = 4

class XYZTranslationTask(AbstractTask):
    def __init__(self, task_request):
        super(XYZTranslationTask, self).__init__()

        self._canceled = False

        self._transition = None

        self._plan = None
        self._feedback = None

        self._complete_time = None
        self._sent_plan_time = None

        self._corrected_start_x = None
        self._corrected_start_y = None
        self._corrected_start_z = None

        self._vel_start = None

        self._lock = threading.RLock()

        self._linear_gen = self.topic_buffer.get_linear_motion_profile_generator()

        try:
            self._PLANNING_TIMEOUT = rospy.Duration(rospy.get_param('~planning_timeout'))
            self._PLANNING_LAG = rospy.Duration(rospy.get_param('~planning_lag'))
            self._COORDINATE_FRAME_OFFSET = rospy.get_param('~planner_coordinate_frame_offset')
            self._DONE_REPLAN_DIST = rospy.get_param('~done_replanning_radius')
            self._MIN_MANEUVER_HEIGHT = rospy.get_param('~min_maneuver_height')
        except KeyError as e:
            rospy.logerr('Could not lookup a parameter for xyztranslation task')
            raise

        # Check that we aren't being requested to go below the minimum maneuver height
        # Error straight out if that's the case.
        if task_request.z_position < self._MIN_MANEUVER_HEIGHT :
            raise ValueError('Requested z height was below the minimum maneuver height')

        self._corrected_goal_x = task_request.x_position + self._COORDINATE_FRAME_OFFSET
        self._corrected_goal_y = task_request.y_position + self._COORDINATE_FRAME_OFFSET
        self._corrected_goal_z = task_request.z_position

        self._state = XYZTranslationTaskState.INIT

    def get_desired_command(self):
        with self._lock:
            if self._canceled:
                return (TaskCanceled(),)

            if self._state == XYZTranslationTaskState.COMPLETING:
                if (rospy.Time.now() + rospy.Duration(.10)) >= self._complete_time:
                    return (TaskDone(),)
                else:
                    return (TaskRunning(), NopCommand())

            expected_time = rospy.Time.now() + self._PLANNING_LAG
            starting = self._linear_gen.expected_point_at_time(expected_time)

            _pose = starting.motion_point.pose.position
            self._vel_start = starting.motion_point.twist.linear

            self._corrected_start_x = _pose.x + self._COORDINATE_FRAME_OFFSET
            self._corrected_start_y = _pose.y + self._COORDINATE_FRAME_OFFSET
            self._corrected_start_z = _pose.z

            _distance_to_goal = math.sqrt(
                        (self._corrected_start_x-self._corrected_goal_x)**2 +
                        (self._corrected_start_y-self._corrected_goal_y)**2)

            if _distance_to_goal < self._DONE_REPLAN_DIST:
                if self._plan is not None:
                    self._state = XYZTranslationTaskState.COMPLETING
                    self._complete_time = self._plan.motion_points[-1].header.stamp
                    return (TaskRunning(), GlobalPlanCommand(self._plan))
                else:
                    rospy.logerr('XYZTranslationTask: Plan is None but we are done')
                    return (TaskFailed(msg='Started too close to goal to do anything'),)

            if self._state == XYZTranslationTaskState.INIT:
                self._sent_plan_time = rospy.Time.now()
                self.topic_buffer.make_plan_request(self._generate_request(expected_time),
                                                    self._feedback_callback)
                self._state = XYZTranslationTaskState.WAITING

            if self._state == XYZTranslationTaskState.PLAN_RECEIVED:
                self.topic_buffer.make_plan_request(self._generate_request(expected_time),
                                                    self._feedback_callback)

                if self._feedback is not None and self._feedback.success:
                    self._state = XYZTranslationTaskState.WAITING
                    self._sent_plan_time = rospy.Time.now()
                    # send LLM the plan we received
                    return (TaskRunning(), GlobalPlanCommand(self._plan))

            if (self._sent_plan_time is not None and
                (rospy.Time.now() - self._sent_plan_time) > self._PLANNING_TIMEOUT):
                return (TaskFailed(msg='XYZ Translate: planner took too long to plan'),)

            return (TaskRunning(), NopCommand())

    def _generate_request(self, expected_time):
        request = PlanGoal()
        request.header.stamp = expected_time

        start = MotionPointStamped()
        start.motion_point.pose.position.x = self._corrected_start_x
        start.motion_point.pose.position.y = self._corrected_start_y
        start.motion_point.pose.position.z = self._corrected_start_z

        start.motion_point.twist.linear.x = self._vel_start.x
        start.motion_point.twist.linear.y = self._vel_start.y
        start.motion_point.twist.linear.z = self._vel_start.z

        goal = MotionPointStamped()
        goal.motion_point.pose.position.x = self._corrected_goal_x
        goal.motion_point.pose.position.y = self._corrected_goal_y
        goal.motion_point.pose.position.z = self._corrected_goal_z

        request.start = start
        request.goal = goal
        return request

    def _feedback_callback(self, status, msg):
        with self._lock:
            self._feedback = msg
            self._plan = msg.plan
            self._state = XYZTranslationTaskState.PLAN_RECEIVED

    def cancel(self):
        rospy.loginfo('XYZTranslationTask canceled')
        self._canceled = True
        return True

    def set_incoming_transition(self, transition):
        self._transition = transition
