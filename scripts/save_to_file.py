#!/usr/bin/env python

import rospy
import tf
import os
import csv
import string
import rosbag

from xsens_data_tf.msg import MultiStates
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion



class statesWriter():
        # todo: add different argus for saving file
    def __init__(self,file_path,file_name):
        self.listener = tf.TransformListener()
        
        # define transform list
        self.human_file_name = os.path.join(file_path,'human-'+file_name+'.csv')
        self.robot_file_name = os.path.join(file_path,'robot-'+file_name+'.csv')
        
        print self.human_file_name
        print self.robot_file_name

        human_bag_file = os.path.join(file_path,'human-'+file_name+'.bag')
        self.human_bag = rosbag.Bag(human_bag_file,'w') 

        rospy.Subscriber('/human_states', MultiStates, self.write_human_data)
        rospy.Subscriber('/robot_states', MultiStates, self.write_robot_data)

    ## TODO: write result to csv file

    def write_human_data(self, data):
        rospy.loginfo(rospy.get_caller_id() + "I heard msg")
        # self.bag.write('/human_states',data)
        with open(self.human_file_name,'a') as csvfile:
            # for trans in data.transform_states:
            self.write_tf_to_csv(data.transform_states,csvfile)



    def write_robot_data(self, data):
        rospy.loginfo(rospy.get_caller_id() + "I heard robot msg")
        

    def write_tf_to_csv(self,trans,csvfile):
        filewriter = csv.writer(csvfile, delimiter = ',')
        firstIteration = True	#allows header row
        t=0

        msgString = str(trans)
        msgList = string.split(msgString, '\n')
        instantaneousListOfData = []
        for nameValuePair in msgList:
            splitPair = string.split(nameValuePair, ':')
            for i in range(len(splitPair)):	#should be 0 to 1
                splitPair[i] = string.strip(splitPair[i])
            instantaneousListOfData.append(splitPair)  
        #write the first row from the first element of each pair
        if firstIteration:	# header
            headers = ["rosbagTimestamp"]	#first column header
            for pair in instantaneousListOfData:
                headers.append(pair[0])
            filewriter.writerow(headers)
            firstIteration = False   
        # write the value from each pair to the file
        t += 1
        values = [str(t)]	#first column will have rosbag timestamp
        for pair in instantaneousListOfData:
            values.append(pair[1])
        # print values
        filewriter.writerow(values)   



def main():

    rospy.init_node('retrans_writer')
    stateswriter = statesWriter('~/Documents/Dataset_10_04/claphand','2018-10-04-21-02-01')
    rospy.spin()

    # rate = rospy.Rate(30.0) #30hz

    # # player_proc = subprocess.Popen(['rosbag', 'play', '2018-10-04-21-02-01.bag'], cwd='/home/xuan/Documents/Dataset_10_04/claphand')
    # while not rospy.is_shutdown():
    #     try:
    #         tf_listener.create_trans_msg()
            
    #     except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #         continue

    #     rate.sleep()


if __name__ == '__main__':
    main()
