#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>

class Controller
{
public:
    Controller(int argc, char **argv)
    {
        ros::init(argc, argv, "ps4_controller_pose_integrated");
        ros::NodeHandle nh;

        sub = nh.subscribe("/joy", 100, &Controller::callback, this);
        pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/stewart/platform_pose", 100);
        twist_pub = nh.advertise<geometry_msgs::Twist>("/stewart/platform_twist", 100);

        pose.pose.position.x = 0.0;
        pose.pose.position.y = 0.0;
        pose.pose.position.z = 0.3;

        roll = pitch = yaw = 0.0;
        last_time = ros::Time::now();
    }

    void run() { ros::spin(); }

private:
    void callback(const sensor_msgs::Joy::ConstPtr& msg)
    {
        ros::Time now = ros::Time::now();
        double dt = (now - last_time).toSec();
        if (dt <= 0.0) dt = 0.01; // 避免除0

        // ========== 从手柄读取速度指令 ==========
        geometry_msgs::Twist twist_msg;
        twist_msg.linear.x = msg->axes[0] * 0.1;   // 速度缩放 0.1 m/s
        twist_msg.linear.y = msg->axes[1] * 0.1;
        twist_msg.linear.z = msg->axes[7] * 0.1; // 上升下降

        twist_msg.angular.x = msg->axes[4] * 0.1;  // 绕X旋转速度
        twist_msg.angular.y = msg->axes[3] * 0.1;  // 绕Y旋转速度
        twist_msg.angular.z = msg->axes[6] * 0.1; // 绕Z旋转速度

        twist_pub.publish(twist_msg); // 仍然发布速度话题（方便调试）

        // ========== 对速度积分得到位姿 ==========
        pose.pose.position.x += twist_msg.linear.x * dt;
        pose.pose.position.y += twist_msg.linear.y * dt;
        pose.pose.position.z += twist_msg.linear.z * dt;

        roll  += twist_msg.angular.x * dt;
        pitch += twist_msg.angular.y * dt;
        yaw   += twist_msg.angular.z * dt;

        // ========== 转换为四元数 ==========
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();

        pose.header.stamp = now;
        pose_pub.publish(pose);

        last_time = now;
    }

    // ROS对象
    ros::Subscriber sub;
    ros::Publisher pose_pub;
    ros::Publisher twist_pub;

    // 位姿状态
    geometry_msgs::PoseStamped pose;
    double roll, pitch, yaw;

    // 时间
    ros::Time last_time;
};

int main(int argc, char **argv)
{
    Controller controller(argc, argv);
    controller.run();
    return 0;
}
