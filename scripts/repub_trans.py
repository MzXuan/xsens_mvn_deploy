#!/usr/bin/env python

import rospy
import tf
import subprocess
import numpy as np


from xsens_data_tf.msg import MultiStates
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion


# ROBOT_TRANS_BASE = '/base'
# ROBOT_TRANS_LIST = ['/left_gripper_base']
# HUMAN_TRANS_BASE = '/body_sensor'
# HUMAN_TRANS_LIST = ['/human_state/pelvis','human_state/t8',
#                         'human_state/right_shoulder','human_state/right_upper_arm',
#                             'human_state/right_forearm','/human_state/right_hand']


ROBOT_TRANS_BASE = '/base'
ROBOT_TRANS_LIST = ['/left_gripper_base','/human_state/right_hand']
HUMAN_TRANS_BASE = '/body_sensor'
HUMAN_TRANS_LIST = ['/human_state/right_hand']


class tfThrottle():

    # todo: add different argus for saving file
    def __init__(self,robot_trans_base,robot_trans_list, human_trans_base, human_trans_list):
        self.listener = tf.TransformListener()
        
        # define transform list
        self.robot_trans_base = robot_trans_base 
        self.robot_trans_list = robot_trans_list

        self.human_trans_base = human_trans_base
        self.human_trans_list = human_trans_list

        # define publihser
        self.human_states_pub = rospy.Publisher('/human_states', MultiStates, queue_size=1)
        self.robot_states_pub = rospy.Publisher('/robot_states', MultiStates, queue_size=1)
        

    def create_trans_msg(self):
        # find certain transofrm
        human_states_msg = MultiStates()
        robot_states_msg = MultiStates()

        trans_temp = TransformStamped()

        time = rospy.Time.now()

        # human's information
        frame_id = HUMAN_TRANS_BASE
        for child_frame_id in HUMAN_TRANS_LIST:
            result = self.find_transform(frame_id, child_frame_id)
            if (result != -1):
                human_states_msg.transform_states.append(result)

        # # robot information
        # frame_id = ROBOT_TRANS_BASE
        # for child_frame_id in ROBOT_TRANS_LIST:
        #     result = self.find_transform(frame_id, child_frame_id)
        #     if (result != -1):
        #         robot_states_msg.transform_states.append(result)


        # robot information
        frame_id = ROBOT_TRANS_BASE
        result = self.robot2hand()
        if (result != -1):
            robot_states_msg.transform_states.append(result)
        
        # print human_states_msg
        # print robot_states_msg

        if ((human_states_msg.transform_states!=[]) and (robot_states_msg.transform_states!=[])):
            # print "successfully publish"
            self.human_states_pub.publish(human_states_msg)
            self.robot_states_pub.publish(robot_states_msg)
        

    
    def robot2hand(self):
        frame_id = '/base'
        child_frame_id_1 = '/left_gripper_base'
        child_frame_id_2 = '/human_state/right_hand'


        trans_temp = TransformStamped()
        trans_temp.header.frame_id = child_frame_id_1
        
        #todo: create message
        trans_temp.child_frame_id = child_frame_id_2


        #This function is for calculate the transform between robot part and human part directly
        try:        
            (trans1, rot1) = self.listener.lookupTransform(frame_id, child_frame_id_1, rospy.Time(0))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            # print("Can not find transform between %s and %s" % (frame_id, child_frame_id))
            return -1

        # change tf transform to matrix
        trans1_mat = tf.transformations.translation_matrix(trans1)
        rot1_mat   = tf.transformations.quaternion_matrix(rot1)
        mat1 = np.dot(trans1_mat, rot1_mat)
        # print("mat1", mat1)


        try:        
            (trans2, rot2) = self.listener.lookupTransform(frame_id, child_frame_id_2, rospy.Time(0))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            # print("Can not find transform between %s and %s" % (frame_id, child_frame_id))
            return -1

        trans2_mat = tf.transformations.translation_matrix(trans2)
        rot2_mat = tf.transformations.quaternion_matrix(rot2)
        mat2 = np.dot(trans2_mat, rot2_mat)
        mat2 = np.linalg.inv(mat2)
        # print("mat2", mat2)

        # multiply the transforms

        mat = np.dot(mat1, mat2)
        trans = tf.transformations.translation_from_matrix(mat)
        rot = tf.transformations.quaternion_from_matrix(mat)

        translation = Vector3()
        translation.x = trans[0]
        translation.y = trans[1]
        translation.z = trans[2]

        rotation = Quaternion()
        rotation.x = rot[0]
        rotation.y = rot[1]
        rotation.z = rot[2]
        rotation.w = rot[3]
            
        trans_temp.transform.translation = translation
        trans_temp.transform.rotation = rotation

        return trans_temp
    
    

    # def multiply_transform(self,trans_list):
    #     #This function is for calculate the transform between robot part and human part directly
    #     trans_stamp1 = trans_list[0]
    #     trans_stamp2 = trans_list[1]

    #     # change tf transform to matrix
    #     trans1_mat = tf.transformations.translation_matrix(trans_stamp1.transform.translation)
    #     rot1_mat   = tf.transformations.quaternion_matrix(trans_stamp1.transform.rotation)
    #     mat1 = np.dot(trans1_mat, rot1_mat)

    #     print("mat1", mat1)

    #     trans2_mat = tf.transformations.translation_matrix(trans_stamp2.transform.translation)
    #     rot2_mat    = tf.transformations.quaternion_matrix(trans_stamp2.transform.rotation)
    #     mat2 = np.dot(trans2_mat, rot2_mat).inv
    #     print("mat2", mat2)

    #     # multiply the transforms

    #     mat3 = np.dot(mat1, mat2)
    #     trans3 = tf.transformations.translation_from_matrix(mat3)
    #     rot3 = tf.transformations.quaternion_from_matrix(mat3)

    #     return 0



    def find_transform(self, frame_id, child_frame_id):
        trans_temp = TransformStamped()
        trans_temp.header.frame_id = frame_id
        
        #todo: create message
        trans_temp.child_frame_id = child_frame_id

        try:        
            (trans, rot) = self.listener.lookupTransform(frame_id, child_frame_id, rospy.Time(0))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            # print("Can not find transform between %s and %s" % (frame_id, child_frame_id))
            return -1
        
        translation = Vector3()
        translation.x = trans[0]
        translation.y = trans[1]
        translation.z = trans[2]

        rotation = Quaternion()
        rotation.x = rot[0]
        rotation.y = rot[1]
        rotation.z = rot[2]
        rotation.w = rot[3]
            
        trans_temp.transform.translation = translation
        trans_temp.transform.rotation = rotation

        return trans_temp
    
            

def main():

    rospy.init_node('turtle_tf_listener')
    tf_listener = tfThrottle(ROBOT_TRANS_BASE,ROBOT_TRANS_LIST, HUMAN_TRANS_BASE, HUMAN_TRANS_LIST)

    rate = rospy.Rate(30.0) #30hz

    # player_proc = subprocess.Popen(['rosbag', 'play', '2018-10-04-21-02-01.bag'], cwd='/home/xuan/Documents/Dataset_10_04/claphand')
    while not rospy.is_shutdown():
        try:
            tf_listener.create_trans_msg()
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()


if __name__ == '__main__':
    main()
