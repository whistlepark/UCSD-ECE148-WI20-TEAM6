#!/usr/bin/env python3
import rospy
import message_filters
import yaml
from rospy_message_converter import message_converter

class ros_wrapper():
    def __init__(self):
        self.time_syncros = []
        self.subs=[]
        self.sync_subs=[]

    def start(self):
        rospy.init_node('wrapper')

    def add_topic(self, name, topic_type, callback):
        self.subs.append(rospy.Subscriber(name, topic_type, self.__handle_msg, (callback)))

    def add_synced_topics(self, names, topic_types, callback):
        subs=[]
        for i in range(len(names)):
            subs.append(message_filters.Subscriber(names[i], topic_types[i]))
        self.time_syncros.append(message_filters.TimeSynchronizer(subs, 10))
        self.time_syncros[-1].registerCallback(self.__handle_sync_msgs, callback)
        self.sync_subs.append(subs)

    def __handle_msg(self, data, callback):
        callback(self.__msg_to_dict(data))

    def __handle_sync_msgs(self, *args):
        msgs=[]
        for i in range(len(args)-1):
            msgs.append(self.__msg_to_dict(args[i]))
        args[-1](*msgs)

    def __msg_to_dict(self, msg):
        return message_converter.convert_ros_message_to_dictionary(msg)
    def spin(self):
        rospy.spin()
