#!/usr/bin/env python3

class ros_wrapper_test():
    def __init__(self):
        self.single_success=False
        self.multi_success=False
    def single_msg_callback(self, msg):
        print('Single message callback received.')
        print(msg)
        self.single_success=True

    def multi_msg_callback(self, msg1, msg2):
        print('Multi message callback received.')
        print(msg1['header'])
        print(msg2['header'])
        self.multi_success=True

    def start_test(self):
        single_msg_topic='/test'
        multi_msg_topics=['/test1', '/test2']
        multi_msg_types =[Odometry, Odometry]

        rw=ros_wrapper()
        print("Adding single message topic")
        rw.add_topic(single_msg_topic, String, self.single_msg_callback)
        print("Adding synced message topics")
        rw.add_synced_topics(multi_msg_topics, multi_msg_types, self.multi_msg_callback)

        single_pub=rospy.Publisher(single_msg_topic, String, queue_size=10)

        multi_pubs=[]
        for i in range(len(multi_msg_topics)):
            multi_pubs.append(rospy.Publisher(multi_msg_topics[i], multi_msg_types[i], queue_size=10))

        rospy.init_node('ros_wrapper_test')

        msg="test single message"
        single_pub.publish(msg)

        msgs=[]
        msg=Odometry()
        msg.header.frame_id="one"
        msgs.append(msg)
        msg=Odometry()
        msg.header.frame_id="two"
        msgs.append(msg)

        for i in range(len(multi_pubs)):
            multi_pubs[i].publish(msgs[i])

    def success(self):
        return self.single_success and self.multi_success

if __name__=='__main__':
    from rospy_wrapper import ros_wrapper
    from std_msgs.msg import String
    from nav_msgs.msg import Odometry
    import time
    import rospy

    tester=ros_wrapper_test()
    print("starting test")
    tester.start_test()
    while not tester.success():
        time.sleep(0.5)
    print("success!")
