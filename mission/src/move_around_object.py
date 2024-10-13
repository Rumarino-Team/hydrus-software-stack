#!/usr/bin/env python3
# This is the example state machine for
# constructing the other machines with their respective 
# missions.

import smach
import rospy
from movement import UpdatePoseState, UpdatePoseToObjectState, ContinuePoseObjectMovement
from geometry_msgs.msg import Point
import os

class GoAroundObject(smach.StateMachine):
    def __init__(self, shared_state_data,object_name):
        smach.StateMachine.__init__(self, outcomes=['finish', 'failure'])
        # shared_data is initialized inisde the initialize_subscribers() function
        # This is a global variable that is shared between all the states.
        self.userdata.shared_data = shared_state_data
        self.starting_point = shared_state_data.zed_data["pose"].pose.position

        with self:
            smach.StateMachine.add('move_left_object',
                                    UpdatePoseToObjectState( edge_case_callback= movement_edge_case_callback,point= Point(x=-0.2,y = 0.2, z =0 ),
                                                            desired_object_name=object_name), 
                                    transitions={'success':'move_right_object', 'aborted':'failure', 'edge_case_detected':'failure', "object_not_detected": "move_left_object"})  
            smach.StateMachine.add("move_right_object", ContinuePoseObjectMovement(Point(x=-0.2,y = 0.2, z =0 ), edge_case_callback=movement_edge_case_callback),
                                                                        transitions={'success':'move_starting_point', 'aborted':'failure', 'edge_case_detected':'failure'})

            smach.StateMachine.add("move_starting_point", UpdatePoseState(self.starting_point, point= Point(x=-0.2,y = 0.2, z =0 )),
                                    transitions={'success':'finish', 'aborted':'failure', 'edge_case_detected':'failure'})
