#include <cmath>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "geometry_msgs/PoseStamped.h"
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"

class IK
{
public: 
    IK(int argc, char **argv)
    {
        height = 2.0;
        b << -0.101,    0.8, 0.25, 1,
              0.101,    0.8, 0.25, 1,
              0.743, -0.313, 0.25, 1,
              0.642, -0.487, 0.25, 1,
             -0.643, -0.486, 0.25, 1,
             -0.744, -0.311, 0.25, 1;

        p << -0.642,  0.487, -0.05, 1,
              0.642,  0.487, -0.05, 1,
              0.743,  0.313, -0.05, 1,
              0.101,   -0.8, -0.05, 1,
             -0.101,   -0.8, -0.05, 1,
             -0.743,  0.313, -0.05, 1;
        for (int i = 0; i < 6; i++)
        {
            f32ma_msg.data.push_back(0);
        }

        ros::init(argc, argv, "ik");
        ros::NodeHandle nh;
        pub = nh.advertise<std_msgs::Float32MultiArray>("/stewart/position_cmd", 100);
        sub = nh.subscribe("stewart/platform_pose", 100, &IK::callback, this);
    }

    void run()
    {
        ros::spin();
    }

private: 
    void callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        float x = msg->pose.position.x;
        float y = msg->pose.position.y;
        float z = msg->pose.position.z;
        // 从四元数提取 roll、pitch、yaw
        Eigen::Quaternionf q(
            msg->pose.orientation.w,
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z
        );
        Eigen::Vector3f euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
        float roll = euler[0];
        float pitch = euler[1];
        float yaw = euler[2];
        Eigen::Matrix<float, 4, 4> T = transformation_matrix(x, y, z + height, roll, pitch, yaw);
        for (size_t i = 0; i < 6; i++)
        {
            Eigen::Matrix<float, 4, 1> length = T*p.row(i).transpose() - b.row(i).transpose();
            f32ma_msg.data[i] = sqrt(pow(length(0), 2) + pow(length(1), 2) + pow(length(2), 2)) - height;
        }
        pub.publish(f32ma_msg);
    }

    Eigen::Matrix<float, 4, 4> transformation_matrix(float x, float y, float z, float r, float p, float yaw)
    {
        Eigen::Matrix<float, 4, 4> T;
        T << cos(yaw)*cos(p), -sin(yaw)*cos(r) + cos(yaw)*sin(p)*sin(r),  sin(yaw)*sin(r)+cos(yaw)*sin(p)*cos(r), x,
             sin(yaw)*cos(p),  cos(yaw)*cos(r) + sin(yaw)*sin(p)*sin(r), -cos(yaw)*sin(r)+sin(yaw)*sin(p)*cos(r), y,
                     -sin(p),                             cos(p)*sin(r),                         cos(p)*cos(yaw), z,
                           0,                                         0,                                       0, 1;
        return T;
    }

    float height;

    Eigen::Matrix<float, 6, 4> b, p;

    ros::Publisher pub;
    ros::Subscriber sub;
    std_msgs::Float32MultiArray f32ma_msg;
};

int main(int argc, char **argv)
{
    IK ik(argc, argv);
    ik.run();

    return 0;
}