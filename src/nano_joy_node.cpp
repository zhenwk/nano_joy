#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <nano_quad_ros/crazyflieCommand.h>

class NanoJoyNode
{
public:
    NanoJoyNode(ros::NodeHandle nh);
    ~NanoJoyNode(){
        nano_quad_ros::crazyflieCommand cmd_msg;
        cmd_msg.roll   = 0.0;
        cmd_msg.pitch  = 0.0;
        cmd_msg.yaw    = 0.0;
        cmd_msg.thrust = 0;
        cmd_msg.stamp  = ros::Time::now();
        cmd_pub.publish(cmd_msg);
    }

private:
    // joy stick callback function
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    double max_roll, max_pitch, max_yaw, max_thrust;

    ros::Publisher cmd_pub;
    ros::Subscriber joy_sub;
};

NanoJoyNode::NanoJoyNode(ros::NodeHandle nh)
{
    // get parameter from launch file
    nh.getParam("max_roll", max_roll);
    nh.getParam("max_pitch", max_pitch);
    nh.getParam("max_yaw", max_yaw);
    nh.getParam("max_thrust", max_thrust);

    // publish crazyflie 2.0 command
    cmd_pub = nh.advertise<nano_quad_ros::crazyflieCommand>("/nano_quad_ros/crazyflieCommand", 1);

    // subscribe to joystick topic
    joy_sub = nh.subscribe<sensor_msgs::Joy>("/joy", 10, &NanoJoyNode::joyCallback, this);
}

void NanoJoyNode::joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
{
    nano_quad_ros::crazyflieCommand cmd_msg;
    double roll, pitch, yaw, thrust;

    roll   = -max_roll   * joy->axes[2];
    pitch  = -max_pitch  * joy->axes[3];
    yaw    =  max_yaw    * joy->axes[0];
    thrust =  max_thrust * joy->axes[1];

    cmd_msg.stamp  = ros::Time::now();
    cmd_msg.roll   = roll;
    cmd_msg.pitch  = pitch;
    cmd_msg.yaw    = yaw;
    cmd_msg.thrust = (int)thrust;

    if(cmd_msg.thrust < 0) cmd_msg.thrust = 0;

    cmd_pub.publish(cmd_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nano_joy_node");
    ros::NodeHandle n("~");
    NanoJoyNode nj(n);

    ros::spin();
}
