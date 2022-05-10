#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Eigen>

using namespace std;

Eigen::Matrix<double,3,4> uav0_abs_pos = Eigen::MatrixXd::Zero(3,4);
Eigen::Matrix<double,3,4> uav1_abs_pos = Eigen::MatrixXd::Zero(3,4);
Eigen::Matrix<double,3,4> uav2_abs_pos = Eigen::MatrixXd::Zero(3,4);
Eigen::Matrix<double,3,4> uav3_abs_pos = Eigen::MatrixXd::Zero(3,4);

void pos00_cb(geometry_msgs::PoseStamped msg){
    uav0_abs_pos.col(0) << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
}

void pos10_cb(geometry_msgs::PoseStamped msg){
    uav0_abs_pos.col(1) << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
}

void pos20_cb(geometry_msgs::PoseStamped msg){
    uav0_abs_pos.col(2) << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
}

void pos30_cb(geometry_msgs::PoseStamped msg){
    uav0_abs_pos.col(3) << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
}

void pos01_cb(geometry_msgs::PoseStamped msg){
    uav1_abs_pos.col(0) << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
}

void pos11_cb(geometry_msgs::PoseStamped msg){
    uav1_abs_pos.col(1) << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
}

void pos21_cb(geometry_msgs::PoseStamped msg){
    uav1_abs_pos.col(2) << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
}

void pos31_cb(geometry_msgs::PoseStamped msg){
    uav1_abs_pos.col(3) << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
}

void pos02_cb(geometry_msgs::PoseStamped msg){
    uav2_abs_pos.col(0) << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
}

void pos12_cb(geometry_msgs::PoseStamped msg){
    uav2_abs_pos.col(1) << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
}

void pos22_cb(geometry_msgs::PoseStamped msg){
    uav2_abs_pos.col(2) << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
}

void pos32_cb(geometry_msgs::PoseStamped msg){
    uav2_abs_pos.col(3) << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
}

void pos03_cb(geometry_msgs::PoseStamped msg){
    uav3_abs_pos.col(0) << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
}

void pos13_cb(geometry_msgs::PoseStamped msg){
    uav3_abs_pos.col(1) << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
}

void pos23_cb(geometry_msgs::PoseStamped msg){
    uav3_abs_pos.col(2) << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
}

void pos33_cb(geometry_msgs::PoseStamped msg){
    uav3_abs_pos.col(3) << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "state_mean");
    ros::NodeHandle nh;

    ros::Subscriber uav0_pos0 = nh.subscribe<geometry_msgs::PoseStamped>("/uav0/abs_position/0",1,pos00_cb);
    ros::Subscriber uav0_pos1 = nh.subscribe<geometry_msgs::PoseStamped>("/uav0/abs_position/1",1,pos01_cb);
    ros::Subscriber uav0_pos2 = nh.subscribe<geometry_msgs::PoseStamped>("/uav0/abs_position/2",1,pos02_cb);
    ros::Subscriber uav0_pos3 = nh.subscribe<geometry_msgs::PoseStamped>("/uav0/abs_position/3",1,pos03_cb);

    ros::Subscriber uav1_pos0 = nh.subscribe<geometry_msgs::PoseStamped>("/uav1/abs_position/0",1,pos10_cb);
    ros::Subscriber uav1_pos1 = nh.subscribe<geometry_msgs::PoseStamped>("/uav1/abs_position/1",1,pos11_cb);
    ros::Subscriber uav1_pos2 = nh.subscribe<geometry_msgs::PoseStamped>("/uav1/abs_position/2",1,pos12_cb);
    ros::Subscriber uav1_pos3 = nh.subscribe<geometry_msgs::PoseStamped>("/uav1/abs_position/3",1,pos13_cb);

    ros::Subscriber uav2_pos0 = nh.subscribe<geometry_msgs::PoseStamped>("/uav2/abs_position/0",1,pos20_cb);
    ros::Subscriber uav2_pos1 = nh.subscribe<geometry_msgs::PoseStamped>("/uav2/abs_position/1",1,pos21_cb);
    ros::Subscriber uav2_pos2 = nh.subscribe<geometry_msgs::PoseStamped>("/uav2/abs_position/2",1,pos22_cb);
    ros::Subscriber uav2_pos3 = nh.subscribe<geometry_msgs::PoseStamped>("/uav2/abs_position/3",1,pos23_cb);

    ros::Subscriber uav3_pos0 = nh.subscribe<geometry_msgs::PoseStamped>("/uav3/abs_position/0",1,pos30_cb);
    ros::Subscriber uav3_pos1 = nh.subscribe<geometry_msgs::PoseStamped>("/uav3/abs_position/1",1,pos31_cb);
    ros::Subscriber uav3_pos2 = nh.subscribe<geometry_msgs::PoseStamped>("/uav3/abs_position/2",1,pos32_cb);
    ros::Subscriber uav3_pos3 = nh.subscribe<geometry_msgs::PoseStamped>("/uav3/abs_position/3",1,pos33_cb);

    ros::Publisher uav0_pos = nh.advertise<geometry_msgs::PoseStamped>("/uav0_position",1);
    ros::Publisher uav1_pos = nh.advertise<geometry_msgs::PoseStamped>("/uav1_position",1);
    ros::Publisher uav2_pos = nh.advertise<geometry_msgs::PoseStamped>("/uav2_position",1);
    ros::Publisher uav3_pos = nh.advertise<geometry_msgs::PoseStamped>("/uav3_position",1);

    Eigen::Matrix<double,3,4> uav_pos_mean = Eigen::MatrixXd::Zero(3,4);
    Eigen::Matrix<double,3,4> initial_pos;

    initial_pos.col(0) << 1,0,0;
    initial_pos.col(1) << 0,1,0;
    initial_pos.col(2) << -1,0,0;
    initial_pos.col(3) << 0,-1,0;

    geometry_msgs::PoseStamped pose0;
    geometry_msgs::PoseStamped pose1;
    geometry_msgs::PoseStamped pose2;
    geometry_msgs::PoseStamped pose3;

    ros::Rate rate(30.0);

    while(ros::ok())
    {
        uav_pos_mean.col(0) = 0.25 * (uav0_abs_pos.col(0) + uav0_abs_pos.col(1) + uav0_abs_pos.col(2) + uav0_abs_pos.col(3));
        uav_pos_mean.col(1) = 0.25 * (uav1_abs_pos.col(0) + uav1_abs_pos.col(1) + uav1_abs_pos.col(2) + uav1_abs_pos.col(3));
        uav_pos_mean.col(2) = 0.25 * (uav2_abs_pos.col(0) + uav2_abs_pos.col(1) + uav2_abs_pos.col(2) + uav2_abs_pos.col(3));
        uav_pos_mean.col(3) = 0.25 * (uav3_abs_pos.col(0) + uav3_abs_pos.col(1) + uav3_abs_pos.col(2) + uav3_abs_pos.col(3));

        // uav_pos_mean = uav_pos_mean + initial_pos;

        pose0.pose.position.x = uav_pos_mean(0,0);
        pose0.pose.position.y = uav_pos_mean(1,0);
        pose0.pose.position.z = uav_pos_mean(2,0);

        pose1.pose.position.x = uav_pos_mean(0,1);
        pose1.pose.position.y = uav_pos_mean(1,1);
        pose1.pose.position.z = uav_pos_mean(2,1);

        pose2.pose.position.x = uav_pos_mean(0,2);
        pose2.pose.position.y = uav_pos_mean(1,2);
        pose2.pose.position.z = uav_pos_mean(2,2);

        pose3.pose.position.x = uav_pos_mean(0,3);
        pose3.pose.position.y = uav_pos_mean(1,3);
        pose3.pose.position.z = uav_pos_mean(2,3);

        pose0.header.stamp.sec = ros::Time::now().sec;
        pose1.header.stamp.sec = ros::Time::now().sec;
        pose2.header.stamp.sec = ros::Time::now().sec;
        pose3.header.stamp.sec = ros::Time::now().sec;

        pose0.header.stamp.nsec = ros::Time::now().nsec;
        pose1.header.stamp.nsec = ros::Time::now().nsec;
        pose2.header.stamp.nsec = ros::Time::now().nsec;
        pose3.header.stamp.nsec = ros::Time::now().nsec;

        uav0_pos.publish(pose0);
        uav1_pos.publish(pose1);
        uav2_pos.publish(pose2);
        uav3_pos.publish(pose3);

        ros::spinOnce();
        rate.sleep();
    }

    
}