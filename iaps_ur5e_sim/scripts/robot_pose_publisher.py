import tf
import rospy
import time
from geometry_msgs.msg import PoseStamped

def pose_publisher():
    pub = rospy.Publisher('robot_pose', PoseStamped, queue_size=10)
    rospy.init_node('robot_pose', anonymous=True)

    rate = rospy.Rate(500)     # 500 Hz

    tf_tcp = tf.TransformListener()
    time.sleep(1.0)

    frameBase = "base"
    frameTarget = "tool0_controller"

    while not rospy.is_shutdown():
        (trans, rot) = tf_tcp.lookupTransform(frameBase, frameTarget, rospy.Time(0))
        
        tcp_pose = PoseStamped()
        curr_time = rospy.Time.now()
        
        tcp_pose.header.stamp.secs = curr_time.secs
        tcp_pose.header.stamp.nsecs = curr_time.nsecs
        tcp_pose.header.frame_id = frameBase

        tcp_pose.pose.position.x = trans[0]
        tcp_pose.pose.position.y = trans[1]
        tcp_pose.pose.position.z = trans[2]

        tcp_pose.pose.orientation.x = rot[0]
        tcp_pose.pose.orientation.y = rot[1]
        tcp_pose.pose.orientation.z = rot[2]
        tcp_pose.pose.orientation.w = rot[3]
        
        pub.publish(tcp_pose)

        rate.sleep()


if __name__ == "__main__":
    try:
        pose_publisher()
    except rospy.ROSInterruptException:
        pass
    


