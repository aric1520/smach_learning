#!/usr/bin/env python
from std_msgs.msg import Empty
import smach_ros
import smach
import rospy
import roslib
#roslib.load_manifest('smach_MonitorState_example')


class bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['bar_succeeded'])

    def execute(self, userdata):
        rospy.sleep(3.0)
        return 'bar_succeeded'


def monitor_cb(ud, msg):
    return False


def main():
    rospy.init_node("monitor_example")

    sm = smach.StateMachine(outcomes=['DONE'])
    with sm:
        smach.StateMachine.add('FOO', smach_ros.MonitorState(
            "/sm_reset", Empty, monitor_cb), transitions={'invalid': 'BAR', 'valid': 'FOO', 'preempted': 'FOO'})
        smach.StateMachine.add('BAR', bar(), transitions={
                               'bar_succeeded': 'FOO'})

    sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
    sis.start()
    sm.execute()
    rospy.spin()
    sis.stop()


if __name__ == "__main__":
    main()
