#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import roslib
import time
import rospy
import smach
import smach_ros
from std_msgs.msg import Empty
from twisted.internet.defer import succeed


# define state Bas
class Bas(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        time.sleep(4)
        rospy.loginfo('Executing state BAS')
        return 'succeeded'

# define state Foo


class Foo(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['succeeded', 'preempted', 'aborted'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state FOO')
        time.sleep(1)
        print "counter:%d" % (self.counter)
        if self.counter < 5:
            self.counter += 1
            time.sleep(3)
            return 'succeeded'
        else:
            return 'aborted'


# define state Bar1
class Bar1(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['succeeded', 'preempted', 'aborted'])
        self.task_name = 'task_bar1'

    def execute(self, userdata):
        if self.preempt_requested():  # 如果暂停，则返回暂停状态
            rospy.loginfo("Preempting %s" % (self.task_name))
            self.recall_preempt()  # 唤醒，终止暂停
            return 'preempted'
        time.sleep(5)
        rospy.loginfo('Executing state BAR1')
        return 'succeeded'

# define state Bar2


class Bar2(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['succeeded', 'preempted', 'aborted'])
        self.task_name = 'bar2'

    def execute(self, userdata):
        if self.preempt_requested():  # 如果暂停，则返回暂停状态
            rospy.loginfo("Preempting %s" % (self.task_name))
            self.recall_preempt()  # 唤醒，终止暂停
            return 'preempted'
        time.sleep(5)
        rospy.loginfo('Executing state BAR2')
        return 'succeeded'


# define state Bar3
class Bar3(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['succeeded', 'preempted', 'aborted'])
        self.task_name = 'task_bar3'

    def execute(self, userdata):
        if self.preempt_requested():  # 如果暂停，则返回暂停状态
            rospy.loginfo("Preempting %s" % (self.task_name))
            self.recall_preempt()  # 唤醒，终止暂停
            return 'preempted'
        time.sleep(5)
        rospy.loginfo('Executing state BAR3')
        return 'succeeded'

# define state Charge


class Charge(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        time.sleep(5)
        rospy.loginfo('Executing state BAS')
        return 'succeeded'


class ConcurrentExample:
    def __init__(self):
        rospy.init_node('smach_example_state_machine')

        self.last_bar_state = None
        # Create the top level SMACH state machine
        self.sm_top = smach.StateMachine(outcomes=['stop'])

        # Open the container
        with self.sm_top:

            smach.StateMachine.add('BAS', Bas(),
                                   transitions={'succeeded': 'ZOMBIE'})

            sm_zombie = smach.StateMachine(
                outcomes=['power_off', 'resume'])
            with sm_zombie:
                smach.StateMachine.add(
                    'PLATFORM_CHECK', PlatformCheck(),
                    transitions={'platform_check_ok': 'PLATFORM_STATE_LISTENING'})
                smach.StateMachine.add(
                    'PLATFORM_STATE_LISTENING', smach_ros.MonitorState(
                        "/platform_resume", Empty, platform_resume_monitor_cb),
                    transitions={
                        'invalid': 'power_off',
                        'valid': 'resume',
                        'preempted': 'power_off'})
            smach.StateMachine.add('ZOMBIE', sm_zombie,
                                   transitions={'power_off': 'END',
                                                'resume': 'BAS'})


def platform_resume_monitor_cb(ud, msg):
    rospy.loginfo("receive platform resume monitor msg")
    return False


class PlatformCheck(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['platform_check_ok'])

    def execute(self, userdata):
        rospy.loginfo('Executing state PlatformCheck')
        rospy.sleep(3.0)
        return 'platform_check_ok'


if __name__ == '__main__':
    rospy.init_node('smach_example_state_machine')

    sm_top = smach.StateMachine(outcomes=['stop'])

    # Open the container
    with sm_top:

        smach.StateMachine.add('BAS', Bas(),
                               transitions={'succeeded': 'ZOMBIE'})

        sm_zombie = smach.StateMachine(
            outcomes=['power_off', 'resume'])
        with sm_zombie:
            smach.StateMachine.add(
                'PLATFORM_CHECK', PlatformCheck(),
                transitions={'platform_check_ok': 'PLATFORM_STATE_LISTENING'})
            smach.StateMachine.add(
                'PLATFORM_STATE_LISTENING', smach_ros.MonitorState(
                    "/platform_resume", Empty, platform_resume_monitor_cb),
                transitions={
                    'valid': 'power_off',
                    'invalid': 'resume',
                    'preempted': 'power_off'})
        smach.StateMachine.add('ZOMBIE', sm_zombie,
                               transitions={'power_off': 'stop',
                                            'resume': 'BAS'})
    sis = smach_ros.IntrospectionServer('smach_server', sm_top, '/SM_ROOT')
    sis.start()
    # Execute SMACH plan
    outcome = sm_top.execute()
    rospy.spin()
    sis.stop()
