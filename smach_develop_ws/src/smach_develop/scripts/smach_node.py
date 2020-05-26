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


def result_cb(data):
    rospy.loginfo("receive monitoresult_cb msg")
    return False


def child_term_cb(outcome_map):
    if outcome_map['DANCE_READY'] == 'invalid':
        return True
    elif outcome_map['GRASP_READY'] == 'invalid':
        return True
    elif outcome_map['NAVI_READY'] == 'invalid':
        return True
    elif outcome_map['SYSTEM_STATUS'] == 'invalid':
        return True
    else:
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
    else:
        return 'no_service'


def sys_child_term_cb(outcome_map):
    if outcome_map['SERIVCING'] == 'invalid':
        return True
    elif outcome_map['PLATFORMSTATE'] == 'invalid':
        return True
    else:
        return False


def sys_out_cb(outcome_map):
    if outcome_map['PLATFORMSTATE'] == 'invalid':
        return 'hardware_error'
    elif outcome_map['SERIVCING'] == 'invalid':
        return 'software_error'
    else:
        return 'hardware_error'


class PlatformCheck(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['platform_check_ok'])

    def execute(self, userdata):
        rospy.loginfo('Executing state PlatformCheck')
        rospy.sleep(3.0)
        return 'platform_check_ok'


def main():
    rospy.init_node('smach_example_state_machine')

    rospy.Subscriber('/motion_result', Empty,
                     result_cb)

    sm_top = smach.StateMachine(outcomes=['END'])

    with sm_top:

        sm_sys_con = smach.Concurrence(
            outcomes=['hardware_error', 'software_error'],
            default_outcome='hardware_error',
            child_termination_cb=sys_child_term_cb,
            outcome_cb=sys_out_cb)
        with sm_sys_con:

            # Create the top level SMACH state machine
            sm_service = smach.StateMachine(outcomes=['software_error'])

            # Open the container
            with sm_service:

                smach.StateMachine.add('HARDWARECHECK', HardWareCheck(),
                                       transitions={'hardware_check_ok': 'IDLE'})

                # Create the sub SMACH state machine
                sm_con = smach.Concurrence(outcomes=['dance_service', 'grasp_service',
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
                                                    'shutdown': 'software_error'})

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
                outcomes=['hardware_error'])
            with sm_platform:
                smach.StateMachine.add(
                    'PLATFORMCHECK', PlatformCheck(),
                    transitions={'platform_check_ok': 'PLATFORM_RUNNING'})
                smach.StateMachine.add(
                    'PLATFORM_RUNNING', smach_ros.MonitorState(
                        "/platform_running_status", Empty, monitor_cb),
                    transitions={
                        'invalid': 'hardware_error',
                        'valid': 'hardware_error',
                        'preempted': 'hardware_error'})
            smach.Concurrence.add('PLATFORMSTATE', sm_platform)
        smach.StateMachine.add('WHOLE_SYSTEM', sm_sys_con,
                               transitions={'hardware_error': 'END',
                                            'software_error': 'END'})

    sis = smach_ros.IntrospectionServer('smach_server', sm_top, '/SM_ROOT')
    sis.start()
    # Execute SMACH plan
    outcome = sm_top.execute()
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
