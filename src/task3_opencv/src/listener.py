#!/usr/bin/env python
import rospy
from wasp_custom_msgs.msg import object_loc

ids = {};

def callback(data):
    global ids

    if str(data.ID) not in ids:
        rospy.loginfo("Callback adds: %s", str(data.ID))
        ids[str(data.ID)] = data
        rospy.loginfo("Callback has: %s", ids.values())

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/object_location", object_loc, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
