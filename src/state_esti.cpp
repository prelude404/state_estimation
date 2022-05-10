#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
/*
state_estimation是CMakeLists中的project name，原本catkin_src/rosmsgs的msg文件复制到了
/home/joy/catkin_ws/src/state_estimation下，并在该文件的CMakeLists.txt中添加了
/home/joy/catkin_ws/src/rosmsgs中的CMakeList的内容，一般运行两次才不报错，可注释主干代码验证
*/
#include <state_estimation/Ranging.h>
#include <Eigen/Eigen>
#include <string>
#include <iostream>
#include <math.h>
#include <random>
#include <ctime>

using namespace std;

Eigen::Vector3d current_acc0(0.0,0.0,0.0);
Eigen::Vector3d current_acc1(0.0,0.0,0.0);
Eigen::Vector3d current_acc2(0.0,0.0,0.0);
Eigen::Vector3d current_acc3(0.0,0.0,0.0);

double dt = 1.0/30.0;
double dt_0, dt_1, dt_2, dt_3;
double t0_0, t0_1, t0_2, t0_3;
double t_0, t_1, t_2, t_3;

int current_range_id;
double current_range;

bool gtd0_flag = false;
bool gtd1_flag = false;
bool gtd2_flag = false;
bool gtd3_flag = false;



Eigen::Matrix<double,3,4> initial_pos = Eigen::MatrixXd::Zero(3,4);
Eigen::Matrix<double,3,4> gtd_pos = Eigen::MatrixXd::Zero(3,4);
Eigen::Matrix<double,3,4> gtd_pre_pos = Eigen::MatrixXd::Zero(3,4);
Eigen::Matrix<double,3,4> gtd_vel = Eigen::MatrixXd::Zero(3,4);

// 采用速度的中值积分提升预测步的准确性
Eigen::Matrix<double,3,4> gtd_vel_pre = Eigen::MatrixXd::Zero(3,4);
Eigen::Matrix<double,3,4> self_to_global;

// call back
void gtd0_cb(geometry_msgs::PoseStamped msg){
    if(!gtd0_flag){
        // return;
        initial_pos.col(0) << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
        initial_pos.col(0) = initial_pos.col(0) + self_to_global.col(0);
        // cout << "initial_pos0: (" << msg.pose.position.x << ", " << msg.pose.position.y << ", " << msg.pose.position.z << ")" << endl;
        // dt_0 = 1.0/30.0;
    }
    gtd_pos.col(0) << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
    // mavros的坐标原点是无人机自身（所以初始位置为(0,0,0)），而非世界坐标系的原点
    gtd_pos.col(0) = gtd_pos.col(0) + self_to_global.col(0);
    // t_0 = double(msg.header.stamp.nsec) / pow(10,9);
    // dt_0 = t_0 - t0_0;
    // gtd_vel.col(0) = (gtd_pos.col(0) - gtd_pre_pos.col(0))/dt;
    gtd_pre_pos.col(0) << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
    // t0_0 = double(msg.header.stamp.nsec) / pow(10,9);
    gtd0_flag = true;
}
void gtd1_cb(geometry_msgs::PoseStamped msg){
    if(!gtd1_flag){
        // return;
        initial_pos.col(1) << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
        initial_pos.col(1) = initial_pos.col(1) + self_to_global.col(1);
        // cout << "initial_pos1: (" << msg.pose.position.x << ", " << msg.pose.position.y << ", " << msg.pose.position.z << ")" <<endl;
        // dt_1 = 1.0/30;
    }
    gtd_pos.col(1) << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
    gtd_pos.col(1) = gtd_pos.col(1) + self_to_global.col(1);
    // t_1 = double(msg.header.stamp.nsec) / pow(10,9);
    // dt_1 = t_1 - t0_1;
    // gtd_vel.col(1) = (gtd_pos.col(1) - gtd_pre_pos.col(1))/dt;
    // gtd_pre_pos.col(1) << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
    // t0_1 = double(msg.header.stamp.nsec) / pow(10,9);
    gtd1_flag = true;
}
void gtd2_cb(geometry_msgs::PoseStamped msg){
    if(!gtd2_flag){
        // return;
        initial_pos.col(2) << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
        initial_pos.col(2) = initial_pos.col(2) + self_to_global.col(2);
        // cout << "initial_pos2: (" << msg.pose.position.x << ", " << msg.pose.position.y << ", " << msg.pose.position.z << ")" <<endl;
        // dt_2 = 1/30;
    }
    gtd_pos.col(2) << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
    gtd_pos.col(2) = gtd_pos.col(2) + self_to_global.col(2);
    // t_2 = double(msg.header.stamp.nsec) / pow(10,9);
    // dt_2 = t_2 - t0_2;
    // gtd_vel.col(2) = (gtd_pos.col(2) - gtd_pre_pos.col(2))/dt;
    gtd_pre_pos.col(2) << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
    // t0_2 = double(msg.header.stamp.nsec) / pow(10,9);
    gtd2_flag = true;
}
void gtd3_cb(geometry_msgs::PoseStamped msg){
    if(!gtd3_flag){
        // return;
        initial_pos.col(3) << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
        initial_pos.col(3) = initial_pos.col(3) + self_to_global.col(3);
        // cout << "initial_pos3: (" << msg.pose.position.x << ", " << msg.pose.position.y << ", " << msg.pose.position.z << ")" <<endl;
        // dt_2 = 1/30;
    }
    gtd_pos.col(3) << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
    gtd_pos.col(3) = gtd_pos.col(3) + self_to_global.col(3);
    // t_3 = double(msg.header.stamp.nsec) / pow(10,9);
    // dt_3 = t_3 - t0_3;
    // gtd_vel.col(3) = (gtd_pos.col(3) - gtd_pre_pos.col(3))/dt;
    gtd_pre_pos.col(3) << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
    // t0_3 = double(msg.header.stamp.nsec) / pow(10,9);
    gtd3_flag = true;
}

void gtdv0_cb(geometry_msgs::TwistStamped msg){
    gtd_vel.col(0) << msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z;
    gtd_vel.col(0) = 0.5 * (gtd_vel_pre.col(0) + gtd_vel.col(0));
    gtd_vel_pre.col(0) << msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z;
}

void gtdv1_cb(geometry_msgs::TwistStamped msg){
    gtd_vel.col(1) << msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z;
    gtd_vel.col(1) = 0.5 * (gtd_vel_pre.col(1) + gtd_vel.col(1));
    gtd_vel_pre.col(1) << msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z;
}

void gtdv2_cb(geometry_msgs::TwistStamped msg){
    gtd_vel.col(2) << msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z;
    gtd_vel.col(2) = 0.5 * (gtd_vel_pre.col(1) + gtd_vel.col(1));
    gtd_vel_pre.col(2) << msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z;
}

void gtdv3_cb(geometry_msgs::TwistStamped msg){
    gtd_vel.col(3) << msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z;
    gtd_vel.col(3) = 0.5 * (gtd_vel_pre.col(1) + gtd_vel.col(1));
    gtd_vel_pre.col(3) << msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z;
}

void imu0_cb(sensor_msgs::Imu msg){
    current_acc0 << msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z;
}
void imu1_cb(sensor_msgs::Imu msg){
    current_acc1 << msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z;
}
void imu2_cb(sensor_msgs::Imu msg){
    current_acc2 << msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z;
}
void imu3_cb(sensor_msgs::Imu msg){
    current_acc3 << msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z;
}

void uwb_cb(state_estimation::Ranging msg){
    current_range_id = msg.anchorId;
    current_range = double(msg.range) / 1000;
}

// generate Gaussian Noise
Eigen::Matrix<double,3,4> add_noise(double std){
    static default_random_engine e(time(0));
    static normal_distribution<double> n(0,std);
    Eigen::Matrix<double,3,4> gaussian = Eigen::MatrixXd::Zero(3,4).unaryExpr([](double dummy){return n(e);});
    return gaussian;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "state_esti");
    ros::NodeHandle nh("~"); //句柄不加（“~”）无法利用launch传参
    
    int AGENT_ID = 0;
    Eigen::Vector3i OTHER_ID(1,2,3);

    nh.getParam("agent",AGENT_ID);
    nh.getParam("other1",OTHER_ID(0));
    nh.getParam("other2",OTHER_ID(1));
    nh.getParam("other3",OTHER_ID(2));

    int p,q,r;
    nh.getParam("p",p);
    nh.getParam("q",q);
    nh.getParam("r",r);

    // truevalue for initialization (position and velocity)
    ros::Subscriber uav0_gtd = nh.subscribe<geometry_msgs::PoseStamped>("/uav0/mavros/local_position/pose",1,gtd0_cb);
    ros::Subscriber uav1_gtd = nh.subscribe<geometry_msgs::PoseStamped>("/uav1/mavros/local_position/pose",1,gtd1_cb);
    ros::Subscriber uav2_gtd = nh.subscribe<geometry_msgs::PoseStamped>("/uav2/mavros/local_position/pose",1,gtd2_cb);
    ros::Subscriber uav3_gtd = nh.subscribe<geometry_msgs::PoseStamped>("/uav3/mavros/local_position/pose",1,gtd3_cb);

    ros::Subscriber uav0_gtd_vel = nh.subscribe<geometry_msgs::TwistStamped>("/uav0/mavros/local_position/velocity_local",1,gtdv0_cb);
    ros::Subscriber uav1_gtd_vel = nh.subscribe<geometry_msgs::TwistStamped>("/uav1/mavros/local_position/velocity_local",1,gtdv1_cb); 
    ros::Subscriber uav2_gtd_vel = nh.subscribe<geometry_msgs::TwistStamped>("/uav2/mavros/local_position/velocity_local",1,gtdv2_cb); 
    ros::Subscriber uav3_gtd_vel = nh.subscribe<geometry_msgs::TwistStamped>("/uav3/mavros/local_position/velocity_local",1,gtdv3_cb);  

    // uwb_sub need to be remapped
    ros::Subscriber imu0_sub = nh.subscribe<sensor_msgs::Imu>("/uav0/mavros/imu/data",1,imu0_cb);
    ros::Subscriber imu1_sub = nh.subscribe<sensor_msgs::Imu>("/uav1/mavros/imu/data",1,imu1_cb);
    ros::Subscriber imu2_sub = nh.subscribe<sensor_msgs::Imu>("/uav2/mavros/imu/data",1,imu2_cb);
    ros::Subscriber imu3_sub = nh.subscribe<sensor_msgs::Imu>("/uav3/mavros/imu/data",1,imu3_cb);
    ros::Subscriber uwb_sub = nh.subscribe<state_estimation::Ranging>("/gtec/gazebo/uwb/ranging/"+to_string(AGENT_ID),1,uwb_cb);

    ros::Publisher pos0_pub = nh.advertise<geometry_msgs::PoseStamped>("/uav"+to_string(AGENT_ID)+"/relative_position/"+to_string(OTHER_ID(0)),1);
    ros::Publisher pos1_pub = nh.advertise<geometry_msgs::PoseStamped>("/uav"+to_string(AGENT_ID)+"/relative_position/"+to_string(OTHER_ID(1)),1);
    ros::Publisher pos2_pub = nh.advertise<geometry_msgs::PoseStamped>("/uav"+to_string(AGENT_ID)+"/relative_position/"+to_string(OTHER_ID(2)),1);

    ros::Publisher abs_pos0_pub = nh.advertise<geometry_msgs::PoseStamped>("/uav"+to_string(AGENT_ID)+"/abs_position/0",1);
    ros::Publisher abs_pos1_pub = nh.advertise<geometry_msgs::PoseStamped>("/uav"+to_string(AGENT_ID)+"/abs_position/1",1);
    ros::Publisher abs_pos2_pub = nh.advertise<geometry_msgs::PoseStamped>("/uav"+to_string(AGENT_ID)+"/abs_position/2",1);
    ros::Publisher abs_pos3_pub = nh.advertise<geometry_msgs::PoseStamped>("/uav"+to_string(AGENT_ID)+"/abs_position/3",1);

    ros::Rate rate(30.0);
    
    Eigen::Matrix<double,3,4> current_acc;
    Eigen::Matrix<double,3,3> relative_Iv = Eigen::MatrixXd::Zero(3,3);
    Eigen::Matrix<double,3,3> relative_vel = Eigen::MatrixXd::Zero(3,3);
    Eigen::Matrix<double,3,3> relative_acc = Eigen::MatrixXd::Zero(3,3);
    Eigen::Vector4d raw_dis;
    Eigen::Vector3d relative_dis;
    Eigen::Vector3d initial_relative_dis;
    Eigen::Matrix<double,3,3> initial_relative_pos;
    Eigen::Matrix<double,3,3> relative_pos;
    Eigen::Vector3d y;
    Eigen::Matrix<double,3,3> error_pos = Eigen::MatrixXd::Zero(3,3);


    initial_relative_pos.col(0) = initial_pos.col(AGENT_ID) - initial_pos.col(OTHER_ID(0));
    initial_relative_pos.col(1) = initial_pos.col(AGENT_ID) - initial_pos.col(OTHER_ID(1));
    initial_relative_pos.col(2) = initial_pos.col(AGENT_ID) - initial_pos.col(OTHER_ID(2));
    relative_pos = initial_relative_pos;

    relative_dis(0) = initial_relative_pos.col(0).norm();
    relative_dis(1) = initial_relative_pos.col(1).norm();
    relative_dis(2) = initial_relative_pos.col(2).norm();
    initial_relative_dis = relative_dis;

    geometry_msgs::PoseStamped pose0;
    geometry_msgs::PoseStamped pose1;
    geometry_msgs::PoseStamped pose2;

    geometry_msgs::PoseStamped abs_pose0;
    geometry_msgs::PoseStamped abs_pose1;
    geometry_msgs::PoseStamped abs_pose2;
    geometry_msgs::PoseStamped abs_pose3;

    bool initial_flag = true;

    Eigen::Matrix<double,3,3> A = Eigen::MatrixXd::Identity(3,3);
    Eigen::Matrix<double,3,3> B = Eigen::MatrixXd::Identity(3,3) * dt;
    // ROS_INFO_THROTTLE(1,"B: %f  dt: %f",B(0,0),dt);

    Eigen::Matrix<double,3,3> P = pow(10,p) * Eigen::MatrixXd::Identity(3,3);
    Eigen::Matrix<double,3,3> Q = pow(10,q) * Eigen::MatrixXd::Identity(3,3);
    Eigen::Vector3d K(0.0,0.0,0.0);
    double R = pow(10,r);

    // use the mean position as the position of the whole swarm
    Eigen::Vector3d abs_swarm(0.0,0.0,0.0);
    Eigen::Vector3d abs_vel(0.0,0.0,0.0);
    Eigen::Vector4d calc_mat;
    Eigen::Matrix<double,3,4> abs_pos = Eigen::MatrixXd::Zero(3,4);
    // calc_mat << 0.25, 0.25, 0.25, 0.25,
    //            -0.75, 0.25, 0.25, 0.25,
    //             0.25,-0.75, 0.25, 0.25,
    //             0.25, 0.25,-0.75, 0.25;

    ros::Time begin = ros::Time::now();
    ros::Time now = ros::Time::now();
    ros::Duration duration;
    double dura;

    double runtime = 0.0;
    double RMSE = 0.0;
    ros::Time RMSE_start = ros::Time::now();
    ros::Time RMSE_end = ros::Time::now();
    bool RMSE_flag = true;

    self_to_global.col(0) << 1,0,0;
    self_to_global.col(1) << 0,1,0;
    self_to_global.col(2) << -1,0,0;
    self_to_global.col(3) << 0,-1,0;

    while(ros::ok())
    {   
        // ROS_INFO_THROTTLE(1,"B: %f  dt: %f",B(0,0),dt);
        //1/30 = 0;1.0/30.0 = 0.033;
        // ROS_INFO_THROTTLE(1,"AgentID: %f  OtherID: %f %f %f",AGENT_ID,a,b,c);
        
        // current_acc.col(0) = current_acc0;
        // current_acc.col(1) = current_acc1;
        // current_acc.col(2) = current_acc2;
        // current_acc.col(3) = current_acc3;
        
        // relative_acc.col(0) = current_acc.col(AGENT_ID) - current_acc.col(OTHER_ID(0));
        // relative_acc.col(1) = current_acc.col(AGENT_ID) - current_acc.col(OTHER_ID(1));
        // relative_acc.col(2) = current_acc.col(AGENT_ID) - current_acc.col(OTHER_ID(2));

        // // can be replaced with optical flow velocity in later work
        // relative_vel = relative_vel + dt * relative_acc;
        // relative_Iv = relative_Iv + dt * relative_vel + 0.5 * pow(dt,2) * relative_acc;

        // add noise to gtd_vel
        // gtd_vel = gtd_vel + add_noise(pow(10,-6)) + add_noise(pow(10,-10)).cwiseAbs();
        gtd_vel = gtd_vel + add_noise(pow(10,-10));

        // using relative_vel as input
        relative_vel.col(0) = gtd_vel.col(AGENT_ID) - gtd_vel.col(OTHER_ID(0));
        relative_vel.col(1) = gtd_vel.col(AGENT_ID) - gtd_vel.col(OTHER_ID(1));
        relative_vel.col(2) = gtd_vel.col(AGENT_ID) - gtd_vel.col(OTHER_ID(2));
        relative_Iv = relative_Iv + dt * relative_vel;

        // calculate absolute position of swarm
        abs_vel = 0.25 * (gtd_vel.col(0) + gtd_vel.col(1) + gtd_vel.col(2) + gtd_vel.col(3));
        abs_swarm = abs_swarm + dt * abs_vel;

        raw_dis(current_range_id) = current_range;
        relative_dis(0) = raw_dis(OTHER_ID(0));
        relative_dis(1) = raw_dis(OTHER_ID(1));
        relative_dis(2) = raw_dis(OTHER_ID(2));

        y(0) =  pow(relative_dis(0),2) - pow(initial_relative_dis(0),2) + pow(relative_Iv.col(0).norm(),2);
        y(1) =  pow(relative_dis(1),2) - pow(initial_relative_dis(1),2) + pow(relative_Iv.col(1).norm(),2);
        y(2) =  pow(relative_dis(2),2) - pow(initial_relative_dis(2),2) + pow(relative_Iv.col(2).norm(),2);
        
        if(gtd0_flag && gtd1_flag && gtd2_flag && gtd3_flag && initial_flag)
        {
            initial_relative_pos.col(0) = initial_pos.col(AGENT_ID) - initial_pos.col(OTHER_ID(0));
            initial_relative_pos.col(1) = initial_pos.col(AGENT_ID) - initial_pos.col(OTHER_ID(1));
            initial_relative_pos.col(2) = initial_pos.col(AGENT_ID) - initial_pos.col(OTHER_ID(2));
            relative_pos = initial_relative_pos;
            
            relative_dis(0) = initial_relative_pos.col(0).norm();
            relative_dis(1) = initial_relative_pos.col(1).norm();
            relative_dis(2) = initial_relative_pos.col(2).norm();
            initial_relative_dis = relative_dis;

            abs_swarm = 0.25 * (initial_pos.col(0) + initial_pos.col(1) + initial_pos.col(2) + initial_pos.col(3));
            
            initial_flag = false;
        }

        // ros::Time now = ros::Time::now();
        // duration = now - begin;
        // dura = duration.toSec();
        // if(dura > 5.0 && dura < 10.0){
        //     abs_swarm = abs_swarm + 0.25 * (0.25 * (gtd_pos.col(0) + gtd_pos.col(1) + gtd_pos.col(2) + gtd_pos.col(3))-abs_swarm);
        // }
        // if(dura >=10.0){begin = ros::Time::now();}

        for(int i=0; i<3; i++)
        {
            Eigen::Matrix<double,3,3> P_temp = P;
            Eigen::Matrix<double,1,3> H = 2 * relative_Iv.col(i).transpose();
            //predict
            relative_pos.col(i) = A * relative_pos.col(i) + B * relative_vel.col(i);
            P = A * P * A.transpose() + Q;
            //updatez
            K = P * H.transpose() / (H*P*H.transpose()+R);
            relative_pos.col(i) = relative_pos.col(i) + K * (y(i) - H * relative_pos.col(i));
            P = P - K * H * P;
            if(i!=2){P = P_temp;}

            // relative_pos.col(i) = gtd_pos.col(AGENT_ID) - gtd_pos.col(OTHER_ID(i));
            error_pos.col(i) = relative_pos.col(i) - (gtd_pos.col(AGENT_ID) - gtd_pos.col(OTHER_ID(i)));
        }

        // ROS_INFO_THROTTLE(1,"dt0: %f, dt1: %f, dt2: %f, dt3: %f", 1/dt_0, 1/dt_1, 1/dt_2, 1/dt_3);

        // pose0.pose.position.x = relative_pos(0,0);
        // pose0.pose.position.y = relative_pos(1,0);
        // pose0.pose.position.z = relative_pos(2,0);
        
        // pose1.pose.position.x = relative_pos(0,1);
        // pose1.pose.position.y = relative_pos(1,1);
        // pose1.pose.position.z = relative_pos(2,1);

        // pose2.pose.position.x = relative_pos(0,2);
        // pose2.pose.position.y = relative_pos(1,2);
        // pose2.pose.position.z = relative_pos(2,2);

        abs_pos.col(AGENT_ID) = abs_swarm + 0.25 * (relative_pos.col(0) + relative_pos.col(1) +relative_pos.col(2));
        abs_pos.col(OTHER_ID(0)) = abs_swarm + 0.25 * (-3 * relative_pos.col(0) + relative_pos.col(1) + relative_pos.col(2));
        abs_pos.col(OTHER_ID(1)) = abs_swarm + 0.25 * (relative_pos.col(0) - 3 * relative_pos.col(1) + relative_pos.col(2));
        abs_pos.col(OTHER_ID(2)) = abs_swarm + 0.25 * (relative_pos.col(0) + relative_pos.col(1) - 3 * relative_pos.col(2));

        // 为了便于和mavros的真值比较，转回以机体为原点的坐标系
        // abs_pos = abs_pos - self_to_global;

        pose0.header.stamp.sec = ros::Time::now().sec;
        pose1.header.stamp.sec = ros::Time::now().sec;
        pose2.header.stamp.sec = ros::Time::now().sec;

        pose0.header.stamp.nsec = ros::Time::now().nsec;
        pose1.header.stamp.nsec = ros::Time::now().nsec;
        pose2.header.stamp.nsec = ros::Time::now().nsec;

        abs_pose0.header.stamp.sec = ros::Time::now().sec;
        abs_pose1.header.stamp.sec = ros::Time::now().sec;
        abs_pose2.header.stamp.sec = ros::Time::now().sec;
        abs_pose3.header.stamp.sec = ros::Time::now().sec;

        abs_pose0.header.stamp.nsec = ros::Time::now().nsec;
        abs_pose1.header.stamp.nsec = ros::Time::now().nsec;
        abs_pose2.header.stamp.nsec = ros::Time::now().nsec;
        abs_pose3.header.stamp.nsec = ros::Time::now().nsec;

        pose0.pose.position.x = error_pos(0,0);
        pose0.pose.position.y = error_pos(1,0);
        pose0.pose.position.z = error_pos(2,0);
        
        pose1.pose.position.x = error_pos(0,1);
        pose1.pose.position.y = error_pos(1,1);
        pose1.pose.position.z = error_pos(2,1);

        pose2.pose.position.x = error_pos(0,2);
        pose2.pose.position.y = error_pos(1,2);
        pose2.pose.position.z = error_pos(2,2);

        

        abs_pose0.pose.position.x = abs_pos(0,0);
        abs_pose0.pose.position.y = abs_pos(1,0);
        abs_pose0.pose.position.z = abs_pos(2,0);

        abs_pose1.pose.position.x = abs_pos(0,1);
        abs_pose1.pose.position.y = abs_pos(1,1);
        abs_pose1.pose.position.z = abs_pos(2,1);

        abs_pose2.pose.position.x = abs_pos(0,2);
        abs_pose2.pose.position.y = abs_pos(1,2);
        abs_pose2.pose.position.z = abs_pos(2,2);

        abs_pose3.pose.position.x = abs_pos(0,3);
        abs_pose3.pose.position.y = abs_pos(1,3);
        abs_pose3.pose.position.z = abs_pos(2,3);

        //calculate RMSE of error in the first 30.0s
        runtime = runtime + 1.0;
        RMSE = sqrt((pow(RMSE,2) * (runtime-1.0) + pow(error_pos.col(0).norm(),2) + pow(error_pos.col(1).norm(),2) + pow(error_pos.col(2).norm(),2)) / runtime);
        RMSE_end = ros::Time::now();
        if((((RMSE_end - RMSE_start).toSec()) > 30.0) && RMSE_flag){
            // cout << "RMSE is: " << RMSE << endl;
            RMSE_flag = false;
        }


        pos0_pub.publish(pose0);
        pos1_pub.publish(pose1);
        pos2_pub.publish(pose2);

        abs_pos0_pub.publish(abs_pose0);
        abs_pos1_pub.publish(abs_pose1);
        abs_pos2_pub.publish(abs_pose2);
        abs_pos3_pub.publish(abs_pose3);

        ros::spinOnce();
        rate.sleep();
    }

}

