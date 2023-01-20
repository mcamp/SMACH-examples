#!/usr/bin/env python3

import rospy
import smach
import smach_ros

from std_msgs.msg import Empty
from search_follow_avoid.msg import WObject
from search_follow_avoid.msg import FollowAction, FollowGoal, AvoidAction, AvoidGoal
from states import MultipleMonitorState

class Search(MultipleMonitorState):
    def __init__(self):
        super().__init__("/object", WObject, outcomes=['red','yellow'])
        self.start = rospy.Publisher('/search/start', Empty, queue_size=10)
        self.stop = rospy.Publisher('/search/stop', Empty, queue_size=10)
    
    def on_start(self):
        self.start.publish()

    def on_exit(self):
        self.stop.publish()

    def on_receive(self, msg):
        if(msg.color == 'red'):
            return 'red'
        elif(msg.color=='yellow'):
            return 'yellow'
        return None

class Wait(MultipleMonitorState):
    def __init__(self):
        super().__init__("/object", WObject, timeout=5, outcomes=['red','yellow'])

    def on_receive(self, msg):
        if(msg.color == 'red'):
            return 'red'
        elif(msg.color=='yellow'):
            return 'yellow'
        return None

class Follow(smach_ros.SimpleActionState):
    def __init__(self):
        smach_ros.SimpleActionState.__init__(self,'follow', FollowAction, goal_cb=self.callback)

    @staticmethod
    def callback(ud, goal):
        return FollowGoal()

class Avoid(smach_ros.SimpleActionState):
    def __init__(self):
        smach_ros.SimpleActionState.__init__(self,'avoid', AvoidAction, goal_cb=self.callback)

    @staticmethod
    def callback(ud, goal):
        return AvoidGoal()

def main():
    rospy.init_node('follow_avoid')

    sm = smach.StateMachine(outcomes=['aborted', 'preempted'])
    with sm:
        smach.StateMachine.add('Search', Search(),
                                transitions={'red':'Follow',
                                            'yellow':'Avoid'})
        smach.StateMachine.add('Follow', Follow(),
                                transitions={'succeeded': 'Wait'})
        smach.StateMachine.add('Avoid', Avoid(),
                                transitions={'succeeded':'Wait'})
        smach.StateMachine.add('Wait', Wait(),
                                transitions={'red':'Follow',
                                            'yellow':'Avoid',
                                            'timeout':'Search'})


   
    sis = smach_ros.IntrospectionServer('task_viz', sm, '/follow_avoid')
    sis.start()
    rospy.sleep(1)
    outcome = sm.execute()
    
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()