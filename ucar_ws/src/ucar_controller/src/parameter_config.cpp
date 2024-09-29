
#include <ros/ros.h>


int main(int argc, char **argv)
{
	int max_vel_x;

    // ROS节点初始化
    ros::init(argc, argv, "parameter_config");

    // 创建节点句柄
    ros::NodeHandle node;
	ros::param::get("/move_base/TebLocalPlannerROS/max_vel_x", max_vel_x);
	ROS_INFO("Get max_vel_x[%d]", max_vel_x);
	    ros::param::set("/move_base/TebLocalPlannerROS/max_vel_x", 0.1);
	    ros::param::get("/move_base/TebLocalPlannerROS/max_vel_x", max_vel_x);
	    ROS_INFO("Get max_vel_x[%d]", max_vel_x);
            sleep(5);
            ros::param::del("/move_base/TebLocalPlannerROS/max_vel_x");
            ros::param::set("/move_base/TebLocalPlannerROS/max_vel_x", 0.9);
	
 //   // 读取背景颜色参数
	//ros::param::get("/background_r", red);
	//ros::param::get("/background_g", green);
	//ros::param::get("/background_b", blue);

	//ROS_INFO("Get Backgroud Color[%d, %d, %d]", red, green, blue);

	//// 设置背景颜色参数
	//ros::param::set("/background_r", 255);
	//ros::param::set("/background_g", 255);
	//ros::param::set("/background_b", 255);

	//ROS_INFO("Set Backgroud Color[255, 255, 255]");

 //   // 读取背景颜色参数
	//ros::param::get("/background_r", red);
	//ros::param::get("/background_g", green);
	//ros::param::get("/background_b", blue);

	//ROS_INFO("Re-get Backgroud Color[%d, %d, %d]", red, green, blue);

	//// 调用服务，刷新背景颜色
	//ros::service::waitForService("/clear");
	//ros::ServiceClient clear_background = node.serviceClient<std_srvs::Empty>("/clear");
	//std_srvs::Empty srv;
	//clear_background.call(srv);
	//
	//sleep(1);

    return 0;
}
