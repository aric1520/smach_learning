#!/usr/bin/env python

import rospy
import smach
import smach_ros
from std_msgs.msg import Empty


# define state Bas
class HardWareCheck(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['hardware_check_ok'])

    def execute(self, userdata):
        rospy.loginfo('Executing state HardWareCheck')
        rospy.sleep(3.0)
        return 'hardware_check_ok'

# define state DanceCheck


class DanceCheck(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['dance_check_ok'])

    def execute(self, userdata):
        rospy.loginfo('Executing state DanceCheck')
        rospy.sleep(3.0)
        return 'dance_check_ok'


class PlatformCheck(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['platform_check_ok'])

    def execute(self, userdata):
        rospy.loginfo('Executing state PlatformCheck')
        rospy.sleep(3.0)
        return 'platform_check_ok'

# define state GraspCheck


class GraspCheck(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['grasp_check_ok'])

    def execute(self, userdata):
        rospy.loginfo('Executing state GraspCheck')
        rospy.sleep(3.0)
        return 'grasp_check_ok'


# define state NaviCheck


class NaviCheck(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['navi_check_ok'])

    def execute(self, userdata):
        rospy.loginfo('Executing state NaviCheck')
        rospy.sleep(3.0)
        return 'navi_check_ok'


def monitor_cb(ud, msg):
    rospy.loginfo("receive monitor msg")
    return False


def platform_monitor_cb(ud, msg):
    rospy.loginfo("receive platform monitor msg")
    return False


def platform_resume_monitor_cb(ud, msg):
    rospy.loginfo("receive platform resume monitor msg")
    return False


def result_cb(data):
    rospy.loginfo("receive monitoresult_cb msg")
    return False


def child_term_cb(outcome_map):
    if outcome_map['DANCE_READY'] == 'invalid' or outcome_map['DANCE_READY'] == 'preempted':
        rospy.loginfo("DANCE_READY child_term_cb")
        return True
    elif outcome_map['GRASP_READY'] == 'invalid' or outcome_map['GRASP_READY'] == 'preempted':
        rospy.loginfo("GRASP_READY child_term_cb")
        return True
    elif outcome_map['NAVI_READY'] == 'invalid' or outcome_map['NAVI_READY'] == 'preempted':
        rospy.loginfo("NAVI_READY child_term_cb")
        return True
    elif outcome_map['SYSTEM_STATUS'] == 'invalid' or outcome_map['NAVI_READY'] == 'preempted':
        rospy.loginfo("SYSTEM_STATUS child_term_cb")
        return True
    else:
        rospy.loginfo("child_term_cb")
        return False


def out_cb(outcome_map):
    if outcome_map['DANCE_READY'] == 'invalid':
        return 'dance_service'
    elif outcome_map['GRASP_READY'] == 'invalid':
        return 'grasp_service'
    elif outcome_map['NAVI_READY'] == 'invalid':
        return 'navi_service'
    elif outcome_map['SYSTEM_STATUS'] == 'invalid':
        return 'shutdown'
    elif (outcome_map['DANCE_READY'] == 'preempted'
          and outcome_map['GRASP_READY'] == 'preempted'
          and outcome_map['NAVI_READY'] == 'preempted'
          and outcome_map['SYSTEM_STATUS'] == 'preempted'):
        return 'shutdown'
    else:
        return 'no_service'


def sys_child_term_cb(outcome_map):
    if outcome_map['SERIVCING'] == 'serivce_error':
        rospy.loginfo('SERIVCING sys_child_term_cb cb platform')
        return True
    elif outcome_map['PLATFORMSTATE'] == 'platform_error':
        rospy.loginfo('PLATFORMSTATE sys_child_term_cb cb platform')
        return True
    else:
        rospy.loginfo('nothing sys_child_term_cb cb platform')
        return False


def sys_out_cb(outcome_map):
    if outcome_map['PLATFORMSTATE'] == 'platform_error':
        rospy.loginfo('sys out cb platform')
        return 'platform_error'
    elif outcome_map['SERIVCING'] == 'serivce_error':
        rospy.loginfo('sys out cb service')
        return 'software_error'
    else:
        rospy.loginfo('sys out default')
        return 'platform_error'


def main():
    rospy.init_node('smach_example_state_machine')

    rospy.Subscriber('/motion_result', Empty,
                     result_cb)

    sm_top = smach.StateMachine(outcomes=['END'])

    with sm_top:

        sm_sys_con = smach.Concurrence(
            outcomes=['platform_error', 'software_error'],
            default_outcome='platform_error',
            child_termination_cb=sys_child_term_cb,
            outcome_cb=sys_out_cb)
        with sm_sys_con:

            # Create the top level SMACH state machine
            sm_service = smach.StateMachine(outcomes=['serivce_error'])

            # Open the container
            with sm_service:

                smach.StateMachine.add('HARDWARECHECK', HardWareCheck(),
                                       transitions={'hardware_check_ok': 'IDLE'})

                # Create the sub SMACH state machine
                sm_con = smach.Concurrence(
                    outcomes=['dance_service', 'grasp_service',
                              'navi_service', 'shutdown',
                              'no_service'],
                    default_outcome='no_service',
                    child_termination_cb=child_term_cb,
                    outcome_cb=out_cb)

                # Open the container
                with sm_con:
                    # Add states to the container
                    # smach.Concurrence.add('FOO', Foo())
                    # smach.Concurrence.add('BAR', Bar())
                    smach.Concurrence.add('DANCE_READY', smach_ros.MonitorState(
                        "/motion_dance", Empty, monitor_cb))
                    smach.Concurrence.add('GRASP_READY', smach_ros.MonitorState(
                        "/motion_grasp", Empty, monitor_cb))
                    smach.Concurrence.add('NAVI_READY', smach_ros.MonitorState(
                        "/motion_navi", Empty, monitor_cb))
                    smach.Concurrence.add('SYSTEM_STATUS', smach_ros.MonitorState(
                        "/system_status", Empty, monitor_cb))

                smach.StateMachine.add('IDLE', sm_con,
                                       transitions={'no_service': 'IDLE',
                                                    'dance_service': 'DANCE',
                                                    'grasp_service': 'GRASP',
                                                    'navi_service': 'NAVIGATE',
                                                    'shutdown': 'serivce_error'})

                sm_dancing = smach.StateMachine(
                    outcomes=['dance_done'])
                with sm_dancing:
                    smach.StateMachine.add('DANCECHECK', DanceCheck(),
                                           transitions={'dance_check_ok': 'DANCING'})
                    smach.StateMachine.add('DANCING', smach_ros.MonitorState(
                        "/motion_result", Empty, monitor_cb),
                        transitions={'invalid': 'dance_done', 'valid': 'dance_done', 'preempted': 'dance_done'})
                smach.StateMachine.add('DANCE', sm_dancing,
                                       transitions={'dance_done': 'IDLE'})

                sm_grasping = smach.StateMachine(
                    outcomes=['grasp_done'])
                with sm_grasping:
                    smach.StateMachine.add('GRASPCHECK', GraspCheck(),
                                           transitions={'grasp_check_ok': 'GRASPING'})
                    smach.StateMachine.add('GRASPING', smach_ros.MonitorState(
                        "/grasp_result", Empty, monitor_cb),
                        transitions={'invalid': 'grasp_done', 'valid': 'grasp_done', 'preempted': 'grasp_done'})
                smach.StateMachine.add('GRASP', sm_grasping,
                                       transitions={'grasp_done': 'IDLE'})

                sm_naving = smach.StateMachine(
                    outcomes=['navi_done'])
                with sm_naving:
                    smach.StateMachine.add(
                        'NAVICHECK', NaviCheck(),
                        transitions={'navi_check_ok': 'NAVING'})
                    smach.StateMachine.add('NAVING', smach_ros.MonitorState(
                        "/navi_result", Empty, monitor_cb),
                        transitions={
                        'invalid': 'navi_done',
                        'valid': 'navi_done',
                        'preempted': 'navi_done'})
                smach.StateMachine.add('NAVIGATE', sm_naving,
                                       transitions={'navi_done': 'IDLE'})
            smach.Concurrence.add('SERIVCING', sm_service)

            sm_platform = smach.StateMachine(
                outcomes=['platform_error'])
            with sm_platform:
                smach.StateMachine.add(
                    'PLATFORMCHECK', PlatformCheck(),
                    transitions={'platform_check_ok': 'PLATFORM_RUNNING'})
                smach.StateMachine.add(
                    'PLATFORM_RUNNING',
                    smach_ros.MonitorState(
                        "/platform_running_status",
                        Empty, platform_monitor_cb),
                    transitions={
                        'invalid': 'platform_error',
                        'valid': 'platform_error',
                        'preempted': 'platform_error'})
            smach.Concurrence.add('PLATFORMSTATE', sm_platform)
        smach.StateMachine.add('WHOLE_SYSTEM', sm_sys_con,
                               transitions={'platform_error': 'ZOMBIE',
                                            'software_error': 'ZOMBIE'})
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
                               transitions={'power_off': 'END',
                                            'resume': 'WHOLE_SYSTEM'})

    sis = smach_ros.IntrospectionServer('smach_server', sm_top, '/SM_ROOT')
    sis.start()
    # Execute SMACH plan
    outcome = sm_top.execute()
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
