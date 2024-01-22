import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class PoseToPath:
    def __init__(self):
        rospy.init_node('pose_to_path')

        self.pose_sub = rospy.Subscriber('/j2n6s300_driver/out/tool_pose', PoseStamped, self.pose_callback)
        self.path_pub = rospy.Publisher('/path', Path, queue_size=10)

        self.path_msg = Path()
        self.path_msg.header.frame_id = 'j2n6s300_link_base'

    def pose_callback(self, msg):
        self.path_msg.poses.append(msg)
        self.path_pub.publish(self.path_msg)

if __name__ == '__main__':
    try:
        toPath = PoseToPath()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
