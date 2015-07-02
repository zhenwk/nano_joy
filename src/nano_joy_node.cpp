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
    // send command 'selectively'
    void sendCommand();

    // check channel switch button
    void start();

private:
    // joy stick callback function
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    // cf_controller callback function
    void cfCallback(const nano_quad_ros::crazyflieCommand::ConstPtr& c);

    double MAX_ROLL, MAX_PITCH, MAX_YAW, MAX_THRUST;
    bool CTL_SWITCH;
    std::string ctl_channel;

    ros::Publisher cmd_pub;
    ros::Subscriber js_sub;
    ros::Subscriber cf_sub;

    // command values
    nano_quad_ros::crazyflieCommand cf;
    nano_quad_ros::crazyflieCommand js;
};

NanoJoyNode::NanoJoyNode(ros::NodeHandle nh)
{
    // get parameter from launch file
    nh.getParam("max_roll",    MAX_ROLL);
    nh.getParam("max_pitch",   MAX_PITCH);
    nh.getParam("max_yaw",     MAX_YAW);
    nh.getParam("max_thrust",  MAX_THRUST);
    nh.getParam("ctl_channel", ctl_channel);

    CTL_SWITCH = true;

    // publish crazyflie 2.0 command
    cmd_pub = nh.advertise<nano_quad_ros::crazyflieCommand>("/nano_joy/crazyflieCommand", 1);

    // subscribe to joystick topic
    js_sub = nh.subscribe<sensor_msgs::Joy>("/joy", 10, &NanoJoyNode::joyCallback, this);

    // subscribe to cf_controller topic
    cf_sub = nh.subscribe<nano_quad_ros::crazyflieCommand>("/cf_controller/crazyflieCommand",
                                                           10, &NanoJoyNode::cfCallback, this);
}

void NanoJoyNode::joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
{
    double roll, pitch, yaw, thrust, sw;

    roll   = -MAX_ROLL   * joy->axes[2];
    pitch  = -MAX_PITCH  * joy->axes[3];
    yaw    =  MAX_YAW    * joy->axes[0];
    thrust =  MAX_THRUST * joy->axes[1];
    sw     =               joy->axes[13];

    js.stamp  = ros::Time::now();
    js.roll   = roll;
    js.pitch  = pitch;
    js.yaw    = yaw;
    js.thrust = (int)thrust;
    if(js.thrust < 0) js.thrust = 0;

    // check if controller should be started
    if(sw < 1.0)
        CTL_SWITCH = true;
    else
        CTL_SWITCH = false;
}

void NanoJoyNode::cfCallback(const nano_quad_ros::crazyflieCommand::ConstPtr &c)
{
    cf = *c;
}

void NanoJoyNode::sendCommand()
{
    nano_quad_ros::crazyflieCommand cmd(js);

    if(CTL_SWITCH)
    {
        if(ctl_channel == "roll"  ) cmd.roll   = cf.roll;
        if(ctl_channel == "pitch" ) cmd.pitch  = cf.pitch;
        if(ctl_channel == "yaw"   ) cmd.yaw    = cf.yaw;
        if(ctl_channel == "thrust") cmd.thrust = cf.thrust;
        ROS_INFO("PID dominates: %s",ctl_channel.c_str());
    }
    else ROS_INFO("PID dominates: (none)");

    cmd_pub.publish(cmd);
}

void NanoJoyNode::start()
{
    ROS_INFO("Press switcher to start:");
    while(CTL_SWITCH)
    {
        ros::spinOnce();
        usleep(10000);
    }
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "nano_joy_node");
    ros::NodeHandle n("~");
    NanoJoyNode nj(n);

    nj.start();

    ros::Rate r(30);
    while(n.ok()){
        nj.sendCommand();

        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
