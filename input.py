import rospy
from std_msgs.msg import Int32

def callback(msg):
    rospy.loginfo(msg)

def input():
    # 初始化一个节点
    rospy.init_node("pyinput", anonymous = True)
    # 创建一个订阅者并订阅
    rospy.Subscriber("topic_en", Int32, callback)
    rospy.spin()

if __name__ == "__main__":
    input()
