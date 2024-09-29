import rospy
from std_srv.srv import Empty

def handle_function(srv):
    rospy.loginfo("response")

def server_srv():
    # 创建一个节点
    rospy.init_node("server_srv")
    # 等待接受客户端的请求
    rospy.Service("greetings", Empty, handle_function)
    rospy.spin()

if __name__ == "__main__":
    server_srv()