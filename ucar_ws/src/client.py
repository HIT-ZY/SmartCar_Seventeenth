import rospy
from std_srvs.srv import Empty

def client_srv():
    # 创建一个节点
    rospy.init_node("client")
    # 等待传回来的应答
    rospy.wait_for_service("greetings")
    rospy.loginfo("request")
    try:
        greetings_client = rospy.ServiceProxy("greetings", Empty)
        greetings_client.call()
    except rospy.ServiceException as e:
        rospy.logwarn("faild: %s"%e)

if __name__ == "__main__":
    client_srv()