#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import roslib
import time
import rospy
import smach
import smach_ros
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
                                   transitions={'succeeded': 'CON'})

            # Create the sub SMACH state machine
            self.sm_con = smach.Concurrence(outcomes=['succeeded', 'preempted', 'aborted'],
                                            default_outcome='aborted',
                                            # outcome_map = {'succeeded':{'FOO':'succeeded'},
                                            #                 'aborted':{'FOO':'aborted'}},
                                            child_termination_cb=self.child_term_cb,
                                            outcome_cb=self.out_cb
                                            )

            # Open the container
            with self.sm_con:
                # Add states to the container
                smach.Concurrence.add('FOO', Foo())

                self.sm_bar = smach.StateMachine(
                    outcomes=['succeeded', 'preempted', 'aborted'])
                with self.sm_bar:
                    smach.StateMachine.add('BAR1', Bar1(),
                                           transitions={'succeeded': 'BAR2', 'preempted': 'preempted'})
                    smach.StateMachine.add('BAR2', Bar2(),
                                           transitions={'succeeded': 'BAR3', 'preempted': 'preempted'})
                    smach.StateMachine.add('BAR3', Bar3(),
                                           transitions={'succeeded': 'succeeded', 'preempted': 'preempted'})
                self.sm_bar.register_transition_cb(
                    self.bar_transition_cb, cb_args=[])
                smach.Concurrence.add('BAR', self.sm_bar)

            smach.StateMachine.add('CON', self.sm_con,
                                   transitions={'succeeded': 'stop',
                                                'aborted': 'stop',
                                                'preempted': 'CHARGE'})

            smach.StateMachine.add('CHARGE', Charge(),
                                   transitions={'succeeded': 'CON'})

         # Create and start the introspection server
        sis = smach_ros.IntrospectionServer(
            'server_name', self.sm_top, '/SM_ROOT')
        sis.start()

        # Execute SMACH plan
        outcome = self.sm_top.execute()
        rospy.spin()
        sis.stop()

    # 状态之间转换的时候会调用该函数。比如BAR1转换到BAR2（或者BAR2转换到BAR3）后，执行该回调函数，
    # 那么活动的状态 active_states 是‘BAR2‘（‘BAR3‘）
    def bar_transition_cb(self, userdata, active_states, *cb_args):
        print active_states  # 注意这里是字符串，活动状态的标识符例如‘BAR’
        self.last_bar_state = active_states

    # gets called when ANY child state terminates,
    # 只要Concurent下的其中一个状态完成，都会出发该回调函数
    def child_term_cb(self, outcome_map):

        # terminate all running states if FOO preempted with outcome 'succeeded'
        if outcome_map['FOO'] == 'succeeded':
            print "child_term_cv:FOO finished"
            if self.last_bar_state is not None:

                self.sm_bar.set_initial_state(
                    self.last_bar_state, smach.UserData())
            return True

        # terminate all running states if BAR preempted
        if outcome_map['BAR'] == 'succeeded' or outcome_map['BAR'] == 'preempted':
            print "child_term_cv:SM_BAR finished"

            return True

        # in all other case, just keep running, don't terminate anything
        return False

    # gets called when ALL child states are terminated，只要Concurrent下的状态都结束了，
    # 调用该函数.注意不是BAR下面的BAR1，BAR2，BAR3的之一完成

    def out_cb(self, outcome_map):
        if outcome_map['FOO'] == 'aborted':
            print "out_cb FOO aborted"
            return 'aborted'
        elif outcome_map['BAR'] == 'preempted':

            print "out_cb BAR preempted"
            return 'preempted'
        elif outcome_map['BAR'] == 'succeeded':
            print "out_cb_BAR succeeded"
            return 'succeeded'


if __name__ == '__main__':
    ConcurrentExample()
