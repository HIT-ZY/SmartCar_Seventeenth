import rospy
from std_msgs.msg import Int32

def output():
    # 初始化一个节点
    rospy.init_node("pyoutput", anonymous = True)
    # 创建一个发布者
    pub = rospy.Publisher("topic_en", Int32, queue_size = 1)
    # 设置发送频率
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        # 进行发布
        pub.publish(1)
        rate.sleep()

if __name__ == "__main__":
    output()
