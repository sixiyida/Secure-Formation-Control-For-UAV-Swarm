#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Int32.h>

#define N_drone 4
#define H_flight_plain 1
#define n_sigma 4
#define switch_interval 10
#define num_dest 8
#define num_obs 1

#define r_bet_uavs 1
#define ra_bet_uavs 0.5
#define k_bet_uavs 0.5
#define eps_bet_uavs 1

#define l_bet_obs 0.5
#define L_bet_obs 1.5

double K[2] = {-0.17, -0.37};
double K_obs[2] = {-0.017, -0.037};


typedef struct{
    double px;
    double vx;
    double py;
    double vy;
}h_t;

h_t get_funh(int k, double t)
{
    
    h_t h_out;
    double r = 1.5;
    double omega = 0.2;
    double pi = 3.1416;
    k = k - 1;
    h_out.px = r * cos(omega * t + 2 * pi * k / 3);
    h_out.vx = -omega * r * sin(omega * t + 2 * pi * k / 3);
    h_out.py = r * sin(omega * t + 2 * pi * k / 3);
    h_out.vy = omega * r * cos(omega * t + 2 * pi * k / 3);
    return h_out;
}

double get_distance(double xi, double yi, double xj, double yj)
{
    return sqrt(pow(xi - xj, 2) + pow(yi - yj, 2));
}

double get_wij(double xi, double yi, double xj, double yj)
{
    double dis = get_distance(xi, yi, xj, yj);
    if (dis < r_bet_uavs)
    {
        return -pow(k_bet_uavs, (ra_bet_uavs - dis)/(ra_bet_uavs - r_bet_uavs)) * log(k_bet_uavs) / (ra_bet_uavs - r_bet_uavs) / dis;
    }   
    return 0;
}

double get_oik(double xi, double yi, double xj, double yj)
{
    double dis = get_distance(xi, yi, xj, yj);
    if ((dis <= L_bet_obs) && (dis > l_bet_obs))
    {
        return 4 * (pow(dis, 2) - pow(L_bet_obs, 2)) * (pow(L_bet_obs, 2) - pow(l_bet_obs, 2)) / pow(pow(dis, 2) - pow(l_bet_obs, 2), 3);
    }   
    return 0;
}

int Wmat[n_sigma][4][4] = {
    {
        {0, 0, 0, 0},
        {1, 0, 1, 0},
        {1, 1, 0, 1},
        {1, 0, 1, 0},
    },
    {
        {0, 0, 0, 0},
        {1, 0, 0, 0},
        {0, 0, 0, 1},
        {1, 0, 1, 0},
    },
    {
        {0, 0, 0, 0},
        {1, 0, 1, 0},
        {0, 1, 0, 1},
        {0, 0, 1, 0},
    },
    {
        {0, 0, 0, 0},
        {0, 0, 1, 0},
        {1, 1, 0, 1},
        {0, 0, 1, 0},
    },
};



double init_pos[N_drone][3] = {
    {0, 0, 2},
    {1, 0, 2}, //x, y, z
    {0, 1, 2},
    {-1, 0, 2},
};

double expect_dest[num_dest][2] = {
    {0, 0},
    {1, 1},
    {1.5, 1.7},
    {2, 2.6},
    {2.5, 3.5},
    {2, 2.6},
    {1.5, 1.7},
    {1, 1},
    
}; //x, y

class Leader {
private:
    ros::ServiceClient arming_client_;
    // 订阅话题和发布话题
    ros::Subscriber state_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber velocity_sub_;
    ros::Publisher control_pub_;
    ros::ServiceClient set_mode_client_;
    double alpha[2] = {-3, -10};// ？
public:
    // 无人机名
    std::string name_;
    // ROS节点句柄
    ros::NodeHandle nh_;
    mavros_msgs::SetMode offb_set_mode;
    mavros_msgs::State current_state_;
    geometry_msgs::PoseStamped current_pose_;
    geometry_msgs::TwistStamped current_velocity_;
    // 构造函数
    Leader(const std::string& name) : name_(name) {
        // 解锁/模式切换客户端


        arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/uav" + name_ + "/mavros/cmd/arming");
        set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/uav" + name_ + "/mavros/set_mode");
        // 初始化ROS订阅者和发布者

        state_sub_ = nh_.subscribe<mavros_msgs::State>("/uav" + name_ + "/mavros/state", 1000, &Leader::stateCallback, this);
        pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/uav" + name_ + "/mavros/local_position/pose", 1000, &Leader::poseCallback, this);
        velocity_sub_ = nh_.subscribe<geometry_msgs::TwistStamped>("/uav" + name_ + "/mavros/local_position/velocity_local", 1000, &Leader::velocityCallback, this);
        control_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("/uav" + name_ + "/mavros/setpoint_raw/local", 1000);


        
        // 请求无人机的状态
    }

    // 析构函数
    ~Leader() {
        // 确保在退出时停止无人机
    }

    // 状态回调函数
    void stateCallback(const mavros_msgs::State::ConstPtr& msg) {
        // 更新无人机状态
        current_state_ = *msg;
    }
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // 更新无人机状态
        current_pose_ = *msg;
    }
    void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        // 更新无人机状态
        current_velocity_ = *msg;
    }
    void stay()
    {
        static int stay_mark = 0;
        static mavros_msgs::PositionTarget stay_pt;
        stay_pt.coordinate_frame = 1;
        stay_pt.type_mask = 0b111111111000;
        if (stay_mark == 0)
        {
            stay_pt.position.x = current_pose_.pose.position.x;
            stay_pt.position.y = current_pose_.pose.position.y;
            stay_pt.position.z = current_pose_.pose.position.z;
            stay_mark = 1;
        }
        control_pub_.publish(stay_pt);
    }
    void takeoff()
    {
        static int takeoff_mark = 0;
        static mavros_msgs::PositionTarget takeoff_pt;
        takeoff_pt.coordinate_frame = 1;
        takeoff_pt.type_mask = 0b111111111000;
        if (takeoff_mark == 0)
        {
            takeoff_pt.position.x = current_pose_.pose.position.x;
            takeoff_pt.position.y = current_pose_.pose.position.y;
            takeoff_pt.position.z = H_flight_plain;
            takeoff_mark = 1;
        }
        control_pub_.publish(takeoff_pt);
    }
    void stop()
    {
        mavros_msgs::PositionTarget stop_pt;
        stop_pt.coordinate_frame = 1;
        stop_pt.type_mask = 0b111111000111;
        stop_pt.velocity.x = 0;
        stop_pt.velocity.y = 0;
        stop_pt.velocity.z = 0;
        control_pub_.publish(stop_pt);
    }
    void land()
    {
        static int land_mark = 0;
        static mavros_msgs::PositionTarget land_pt;
        land_pt.coordinate_frame = 1;
        land_pt.type_mask = 0b111111000111;
        if (current_pose_.pose.position.z > 0.25)
        {
            //ROS_INFO("[%d]:land_mark has been set to 1", num_);
            land_pt.velocity.x = 0;
            land_pt.velocity.y = 0;
            land_pt.velocity.z = -0.2;
            //land_mark[num_] = 1;
        }
        else
        {
            land_pt.velocity.x = 0;
            land_pt.velocity.y = 0;
            land_pt.velocity.z = -0.8;
        }
        control_pub_.publish(land_pt);
    }
    void step(int k)
    {
        static double intergral_ctl_input[2] = {0, 0};
        static double double_int_ctl_input[2] = {0, 0};
        mavros_msgs::PositionTarget step_pt;
        step_pt.coordinate_frame = 1;
        step_pt.type_mask = 0b110000111011;
        //step_pt.type_mask = 0b111111111000;
        step_pt.acceleration_or_force.x = alpha[0] * (current_pose_.pose.position.x - expect_dest[k][0]) + alpha[1] * current_velocity_.twist.linear.x;
        step_pt.acceleration_or_force.y = alpha[0] * (current_pose_.pose.position.y - expect_dest[k][1]) + alpha[1] * current_velocity_.twist.linear.y;
        step_pt.acceleration_or_force.z = 0;
        //step_pt.position.z = H_flight_plain;
        //intergral_ctl_input[0] += (alpha[0] * current_pose_.pose.position.x + alpha[1] * current_velocity_.twist.linear.x) * 0.01;
        //intergral_ctl_input[1] += (alpha[0] * current_pose_.pose.position.y + alpha[1] * current_velocity_.twist.linear.y) * 0.01;    
        //double_int_ctl_input[0] += intergral_ctl_input[0] * 0.01;
        //double_int_ctl_input[1] += intergral_ctl_input[1] * 0.01;
        //step_pt.position.x = double_int_ctl_input[0];
        //step_pt.position.y = double_int_ctl_input[1];
        //step_pt.position.z = H_flight_plain;
        //step_pt.position.x = 0;
        //step_pt.position.y = 0;
        step_pt.position.z = H_flight_plain;
        control_pub_.publish(step_pt);
    }
    void setOffboard()
    {
        offb_set_mode.request.custom_mode = "OFFBOARD";
        if( current_state_.mode != "OFFBOARD")
        {
            //客户端set_mode_client向服务端offb_set_mode发起请求call，然后服务端回应response将模式返回，这就打开了offboard模式
            if(set_mode_client_.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                //打开Offboard模式后在终端打印信息
                ROS_INFO("[%s]: Offboard enabled", name_.c_str());
            }
        }
    }
    bool checkOffboard()
    {
        if( current_state_.mode == "OFFBOARD")
            return true;
        else
            return false;
    }
};

class Follower {
private:
    ros::ServiceClient arming_client_;
    // 订阅话题和发布话题
    ros::Subscriber state_sub_;
    ros::Subscriber obs_pos_sub_[num_obs];
    ros::Subscriber pose_sub_[N_drone];
    ros::Subscriber velocity_sub_[N_drone];
    ros::Publisher control_pub_;
    ros::ServiceClient set_mode_client_;
    double alpha[2] = {-3, -10};
public:
    // 无人机名
    std::string name_;
    int num_;
    // ROS节点句柄
    ros::NodeHandle nh_;
    mavros_msgs::SetMode offb_set_mode;
    mavros_msgs::State current_state_;
    geometry_msgs::PoseStamped current_pose_[N_drone], obs_pos[num_obs];
    geometry_msgs::TwistStamped current_velocity_[N_drone];

    // 构造函数
    Follower(const std::string& name) : name_(name) {
        // 解锁/模式切换客户端
        num_ = std::stoi(name_) - 1;
        arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/uav" + name_ + "/mavros/cmd/arming");
        set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/uav" + name_ + "/mavros/set_mode");
        // 初始化ROS订阅者和发布者

        state_sub_ = nh_.subscribe<mavros_msgs::State>("/uav" + name_ + "/mavros/state", 1000, &Follower::stateCallback, this);
        
        pose_sub_[0] = nh_.subscribe<geometry_msgs::PoseStamped>("/uav1/mavros/local_position/pose", 1000, &Follower::poseCallback1, this);
        velocity_sub_[0] = nh_.subscribe<geometry_msgs::TwistStamped>("/uav1/mavros/local_position/velocity_local", 1000, &Follower::velocityCallback1, this);
        pose_sub_[1] = nh_.subscribe<geometry_msgs::PoseStamped>("/uav2/mavros/local_position/pose", 1000, &Follower::poseCallback2, this);
        velocity_sub_[1] = nh_.subscribe<geometry_msgs::TwistStamped>("/uav2/mavros/local_position/velocity_local", 1000, &Follower::velocityCallback2, this);
        pose_sub_[2] = nh_.subscribe<geometry_msgs::PoseStamped>("/uav3/mavros/local_position/pose", 1000, &Follower::poseCallback3, this);
        velocity_sub_[2] = nh_.subscribe<geometry_msgs::TwistStamped>("/uav3/mavros/local_position/velocity_local", 1000, &Follower::velocityCallback3, this);
        pose_sub_[3] = nh_.subscribe<geometry_msgs::PoseStamped>("/uav4/mavros/local_position/pose", 1000, &Follower::poseCallback4, this);
        velocity_sub_[3] = nh_.subscribe<geometry_msgs::TwistStamped>("/uav4/mavros/local_position/velocity_local", 1000, &Follower::velocityCallback4, this);
    
        obs_pos_sub_[0] = nh_.subscribe<geometry_msgs::PoseStamped>("/obstacle1/pose", 1000, &Follower::obsposCallback1, this);
        
        control_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("/uav" + name_ + "/mavros/setpoint_raw/local", 1000);
        // 请求无人机的状态
    }

    // 析构函数
    ~Follower() {
        // 确保在退出时停止无人机
    }

    // 状态回调函数
    void stateCallback(const mavros_msgs::State::ConstPtr& msg) {
        // 更新无人机状态
        current_state_ = *msg;
    }

    void poseCallback1(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // 更新无人机状态
        current_pose_[0] = *msg;
    }
    void velocityCallback1(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        // 更新无人机状态
        current_velocity_[0] = *msg;
    }
    void poseCallback2(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // 更新无人机状态
        current_pose_[1] = *msg;
    }
    void velocityCallback2(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        // 更新无人机状态
        current_velocity_[1] = *msg;
    }
    void poseCallback3(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // 更新无人机状态
        current_pose_[2] = *msg;
    }
    void velocityCallback3(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        // 更新无人机状态
        current_velocity_[2] = *msg;
    }
    void poseCallback4(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // 更新无人机状态
        current_pose_[3] = *msg;
    }
    void velocityCallback4(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        // 更新无人机状态
        current_velocity_[3] = *msg;
    }
    void obsposCallback1(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // 更新无人机状态
        obs_pos[0] = *msg;
    }
    void obsposCallback2(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // 更新无人机状态
        obs_pos[1] = *msg;
    }
    void obsposCallback3(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // 更新无人机状态
        obs_pos[2] = *msg;
    }
    void obsposCallback4(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // 更新无人机状态
        obs_pos[3] = *msg;
    }

   void stay()
    {
        static int stay_mark[4] = {0, 0, 0, 0};
        static mavros_msgs::PositionTarget stay_pt[4];
        stay_pt[num_].coordinate_frame = 1;
        stay_pt[num_].type_mask = 0b111111111000;
        if (stay_mark[num_] == 0)
        {
            //ROS_INFO("[%d]:stay_mark has been set to 1", num_);
            stay_pt[num_].position.x = current_pose_[num_].pose.position.x;
            stay_pt[num_].position.y = current_pose_[num_].pose.position.y;
            stay_pt[num_].position.z = current_pose_[num_].pose.position.z;
            stay_mark[num_] = 1;
        }
        control_pub_.publish(stay_pt[num_]);
        //ROS_INFO("[%d]:Current Target Pos: ([%f],[%f])", num_, current_pose_[num_].pose.position.x, current_pose_[num_].pose.position.y);
        //ROS_INFO("[%d]:Current Target Pos: ([%f],[%f])", num_, stay_pt[num_].position.x, stay_pt[num_].position.y);
    }
    void takeoff()
    {
        static int takeoff_mark[4] = {0, 0, 0, 0};
        static mavros_msgs::PositionTarget takeoff_pt[4];
        takeoff_pt[num_].coordinate_frame = 1;
        takeoff_pt[num_].type_mask = 0b111111111000;
        if (takeoff_mark[num_] == 0)
        {
            //ROS_INFO("[%d]:takeoff_mark has been set to 1", num_);
            takeoff_pt[num_].position.x = current_pose_[num_].pose.position.x;
            takeoff_pt[num_].position.y = current_pose_[num_].pose.position.y;
            takeoff_pt[num_].position.z = H_flight_plain;
            takeoff_mark[num_] = 1;
        }
        ROS_INFO("[%d]:Current Target Pos: ([%f],[%f])", num_, takeoff_pt[num_].position.x, takeoff_pt[num_].position.y);
        control_pub_.publish(takeoff_pt[num_]);
    }
    void stop()
    {
        mavros_msgs::PositionTarget stop_pt;
        stop_pt.coordinate_frame = 1;
        stop_pt.type_mask = 0b111111000111;
        stop_pt.velocity.x = 0;
        stop_pt.velocity.y = 0;
        stop_pt.velocity.z = 0;
        control_pub_.publish(stop_pt);
    }
    void land()
    {
        static int land_mark[4] = {0, 0, 0, 0};
        static mavros_msgs::PositionTarget land_pt[4];
        land_pt[num_].coordinate_frame = 1;
        land_pt[num_].type_mask = 0b111111000111;
        if (current_pose_[num_].pose.position.z > 0.25)
        {
            //ROS_INFO("[%d]:land_mark has been set to 1", num_);
            land_pt[num_].velocity.x = 0;
            land_pt[num_].velocity.y = 0;
            land_pt[num_].velocity.z = -0.2;
            //land_mark[num_] = 1;
        }
        else
        {
            land_pt[num_].velocity.x = 0;
            land_pt[num_].velocity.y = 0;
            land_pt[num_].velocity.z = -0.8;
        }
        //ROS_INFO("[%d]:Current Target Pos: ([%f],[%f])", num_, land_pt[num_].position.x, land_pt[num_].position.y);
        control_pub_.publish(land_pt[num_]);
    }
    void step(int sigma, double t, int k)
    {
        static double last_t;
        double ux = 0, uy = 0;
        double wij = 0;
        double oik = 0;
        ux += alpha[0] * (current_pose_[num_].pose.position.x - expect_dest[k][0]) + alpha[1] * current_velocity_[num_].twist.linear.x;
        uy += alpha[0] * (current_pose_[num_].pose.position.y - expect_dest[k][1]) + alpha[1] * current_velocity_[num_].twist.linear.y;
        for (int i = 1; i < N_drone; i ++)
        {
            if (Wmat[sigma][num_][i])
            {
                ux += K[0] * ((current_pose_[num_].pose.position.x - get_funh(num_, t).px - expect_dest[k][0]) - (current_pose_[i].pose.position.x - get_funh(i, t).px - expect_dest[k][0]));
                ux += K[1] * ((current_velocity_[num_].twist.linear.x - get_funh(num_, t).vx) - (current_velocity_[i].twist.linear.x - get_funh(i, t).vx));
                uy += K[0] * ((current_pose_[num_].pose.position.y - get_funh(num_, t).py - expect_dest[k][1]) - (current_pose_[i].pose.position.y - get_funh(i, t).py - expect_dest[k][1]));
                uy += K[1] * ((current_velocity_[num_].twist.linear.y - get_funh(num_, t).vy) - (current_velocity_[i].twist.linear.y - get_funh(i, t).vy));               

            }
            if (num_ != i)
            {
                wij = get_wij(current_pose_[num_].pose.position.x, current_pose_[num_].pose.position.y, current_pose_[i].pose.position.x, current_pose_[i].pose.position.y);
                ux -= eps_bet_uavs * wij * (current_pose_[num_].pose.position.x - current_pose_[i].pose.position.x);
                uy -= eps_bet_uavs * wij * (current_pose_[num_].pose.position.y - current_pose_[i].pose.position.y);
            }
            
        }
        for (int i = 0; i < num_obs; i ++)
        {
            oik = get_oik(current_pose_[num_].pose.position.x, current_pose_[num_].pose.position.y, obs_pos[i].pose.position.x, obs_pos[i].pose.position.y);
            ux += K_obs[0] * oik * (current_pose_[num_].pose.position.x - obs_pos[i].pose.position.x);
            uy += K_obs[0] * oik * (current_pose_[num_].pose.position.y - obs_pos[i].pose.position.y);
        }
        if (Wmat[sigma][num_][0])
        {
            ux += K[0] * ((current_pose_[num_].pose.position.x - get_funh(num_, t).px - expect_dest[k][0]) - (current_pose_[0].pose.position.x - expect_dest[k][0]));
            ux += K[1] * ((current_velocity_[num_].twist.linear.x - get_funh(num_, t).vx) - (current_velocity_[0].twist.linear.x));
            uy += K[0] * ((current_pose_[num_].pose.position.y - get_funh(num_, t).py - expect_dest[k][1]) - (current_pose_[0].pose.position.y - expect_dest[k][1]));
            uy += K[1] * ((current_velocity_[num_].twist.linear.y - get_funh(num_, t).vy) - (current_velocity_[0].twist.linear.y));               
        }
        
        wij = get_wij(current_pose_[num_].pose.position.x, current_pose_[num_].pose.position.y, current_pose_[0].pose.position.x, current_pose_[0].pose.position.y);
        ux -= eps_bet_uavs * wij * (current_pose_[num_].pose.position.x - current_pose_[0].pose.position.x);
        uy -= eps_bet_uavs * wij * (current_pose_[num_].pose.position.y - current_pose_[0].pose.position.y);

        ux -= alpha[0] * get_funh(num_, t).px + alpha[1] * get_funh(num_, t).vx;
        uy -= alpha[0] * get_funh(num_, t).py + alpha[1] * get_funh(num_, t).vy;
        //ux += (get_funh(num_, t).vx - get_funh(num_, last_t).vx) / (t - last_t);
        //uy += (get_funh(num_, t).vy - get_funh(num_, last_t).vy) / (t - last_t);

        ROS_INFO("UAV%d: ux(%f) uy:(%f)", num_+1, ux, uy);

        mavros_msgs::PositionTarget step_pt;
        step_pt.coordinate_frame = 1;
        step_pt.type_mask = 0b110000111011;
        step_pt.acceleration_or_force.x = ux;
        step_pt.acceleration_or_force.y = uy;
        step_pt.acceleration_or_force.z = 0;
        step_pt.position.z = H_flight_plain;
        control_pub_.publish(step_pt);

    }

    void setOffboard()
    {
        offb_set_mode.request.custom_mode = "OFFBOARD";
        if( current_state_.mode != "OFFBOARD")
        {
            //客户端set_mode_client向服务端offb_set_mode发起请求call，然后服务端回应response将模式返回，这就打开了offboard模式
            if(set_mode_client_.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                //打开Offboard模式后在终端打印信息
                ROS_INFO("[%s]: Offboard enabled", name_.c_str());
            }
        }
    }
    bool checkOffboard()
    {
        if( current_state_.mode == "OFFBOARD")
            return true;
        else
            return false;
    }
};

int state = 0;
int dest_k = 0;

void commandCallback(const std_msgs::Int32::ConstPtr& msg) {
    std_msgs::Int32 current_command; 
    // 更新无人机状态
    current_command = *msg;
    if (current_command.data == 1)
    {
        ROS_INFO("Current state: Takeoff");
        state = 1;
    }
    else if (current_command.data == 9)
    {
        ROS_INFO("Current state: Landing");
        state = 9;
    }
    else if (current_command.data == 2)
    {
        ROS_INFO("Current state: Running Algorithm");
        state = 2;
    }
    else if (current_command.data == 3)
    {
        ROS_INFO("Current state: Stop Moving");
        state = 3;
    }
    else if (current_command.data == 8)
    {
        if (state == 2)
        {
            if (dest_k >= num_dest - 1)
                dest_k = 0;
            else
                dest_k ++;
        }
        else 
            ROS_INFO("Please run algorithm first");
    }
}

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "algorithm");
    // 创建ROS节点句柄
    ros::NodeHandle nh;
    ros::Rate rate(10);
    ros::Rate algorithm_rate(100);
    ros::Subscriber command_sub;

    command_sub = nh.subscribe<std_msgs::Int32>("/command", 1000, &commandCallback);

    int algorithm_startmark = 0;
    int sigma = 0;

    // 创建无人机对象
    Leader UAV0("1");
    Follower UAV1("2");
    Follower UAV2("3");
    Follower UAV3("4");

    ROS_INFO("Launch successfully");

    for(int i = 100; ros::ok() && i > 0; --i){
        UAV0.stop();
        UAV1.stop();
        UAV2.stop();
        UAV3.stop();
        rate.sleep();
        ros::spinOnce();
    }
    ROS_INFO("Having sent pre-offboard messages successfully");
    state = 1;
//    ROS_INFO("Default state: Stay");
    // 确保在ROS节点初始化之后调用ros::Time::now()
    ros::Time start_time, current_time, switch_time;
    float exp_time, from_switch_time;
    while(ros::ok()) {
        //ROS_INFO("Current time: %f", exp_time);
        if (true)
        //if (UAV0.checkOffboard())
        {
            if (state == 0)
            {
                UAV0.stay();
                UAV1.stay();
                UAV2.stay();
                UAV3.stay();
                rate.sleep();
            }    
            if (state == 1)
            {
                UAV0.takeoff();
                UAV1.takeoff();
                UAV2.takeoff();
                UAV3.takeoff();
                rate.sleep();
            }
            else if (state == 2)
            {
                if (algorithm_startmark == 0)
                {
                    switch_time = ros::Time::now();
                    start_time = ros::Time::now();
                    algorithm_startmark = 1;
                    ROS_INFO("Algorithm Start Running");
                    ROS_INFO("Control Rate has been set to 100Hz");

                }
                exp_time = ros::Time::now().toSec() - start_time.toSec();
                from_switch_time = ros::Time::now().toSec() - switch_time.toSec();
                if (from_switch_time >= switch_interval)
                {
                    sigma = 0;
                    switch_time = ros::Time::now();
                }
                ROS_INFO("Running for: %fs, sigma now: %d, destination now: (%f, %f)", exp_time, sigma, expect_dest[dest_k][0], expect_dest[dest_k][1]);
                UAV0.step(dest_k);
                //UAV0.takeoff();
                UAV1.step(sigma, exp_time, dest_k);
                UAV2.step(sigma, exp_time, dest_k);
                UAV3.step(sigma, exp_time, dest_k);
                algorithm_rate.sleep();
            }
            else if (state == 9)
            {
                UAV0.land();
                UAV1.land();
                UAV2.land();
                UAV3.land();
                rate.sleep();

            }
            else if (state == 3)
            {
                UAV0.stop();
                UAV1.stop();
                UAV2.stop();
                UAV3.stop();
                rate.sleep();
            }
        }
        else
        {
            ROS_INFO("Sending pre-offboard messages");
            for (int i = 100; ros::ok() && i > 0; --i)
            {
                UAV0.stop();
                UAV1.stop();
                UAV2.stop();
                UAV3.stop();
                rate.sleep();
                ros::spinOnce();
            }
            ROS_INFO("Trying to turning offboard");
            UAV0.setOffboard();
            UAV1.setOffboard();
            UAV2.setOffboard();
            UAV3.setOffboard();
            ROS_INFO("Current state: Stay");
            state = 0;
            rate.sleep();
        }
        ros::spinOnce();
    }
    return 0;
}