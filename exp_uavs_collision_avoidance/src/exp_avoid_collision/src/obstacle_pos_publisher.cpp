#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#define num_obs 3

class Obstacle {
private:
    ros::Publisher pos_pub_;
public:
    // 无人机名
    std::string name_;
    double pos_x_;
    double pos_y_;
    // ROS节点句柄
    ros::NodeHandle nh_;
    // 构造函数
    Obstacle(const std::string& name, const double &pos_x, const double &pos_y) : name_(name), pos_x_(pos_x), pos_y_(pos_y) {
        // 解锁/模式切换客户端
        pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/obstacle" + name_ , 1000);
        // 请求无人机的状态
    }

    // 析构函数
    ~Obstacle() {
        // 确保在退出时停止无人机
    }
    void pub()
    {
        geometry_msgs::PoseStamped pos;
        pos.pose.position.x = pos_x_;
        pos.pose.position.y = pos_y_;
        pos.pose.position.z = 0.0;
        pos_pub_.publish(pos);
    }
};

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "obs_pos_publisher");
    // 创建ROS节点句柄
    ros::NodeHandle nh;
    ros::Rate rate(10);   
    Obstacle obs1("1", 4, 0); 
    Obstacle obs2("2", 0, 4);
    Obstacle obs3("3", 2, 3);
    ROS_INFO("Publishing Obstacles' Position");

    while (ros::ok())
    {
        obs1.pub();
        obs2.pub();
        obs3.pub();
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}