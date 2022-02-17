#!/usr/bin/env python
import rospy
import message_filters
from std_msgs.msg import Float32
from tf2_msgs.msg import TFMessage

rospy.init_node('synchroniser_node')

pub_force = rospy.Publisher('force_sync', Float32, queue_size=10)
pub_tool_tf = rospy.Publisher('pose_sync', TFMessage, queue_size=10)

def callback(sensor, pose):
  pub_force.publish(sensor)
  #pub_table_tf.publish(table)
  pub_tool_tf.publish(pose)

sub_force = message_filters.Subscriber('sensor', Float32)
sub_pose = message_filters.Subscriber('tf', TFMessage)

ts = message_filters.ApproximateTimeSynchronizer([sub_force, sub_pose], 100, 0.1, 
	allow_headerless=True)
ts.registerCallback(callback)

rospy.spin()