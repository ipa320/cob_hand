#include <cob_hand_bridge/InitFinger.h>
#include <cob_hand_bridge/Status.h>

#include <std_srvs/Trigger.h>

#include <sensor_msgs/JointState.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <diagnostic_updater/publisher.h>

#include <actionlib/server/simple_action_server.h>

#include <ros/ros.h>

#include <boost/thread/mutex.hpp>
#include <boost/scoped_ptr.hpp>

#include <angles/angles.h>

boost::mutex g_mutex;

cob_hand_bridge::Status::ConstPtr g_status; 
boost::scoped_ptr<cob_hand_bridge::JointValues> g_command;

ros::Publisher g_js_pub;
sensor_msgs::JointState g_js;

ros::ServiceClient g_init_finger_client;

ros::Publisher g_command_pub;

void statusCallback(const cob_hand_bridge::Status::ConstPtr& msg){
    boost::mutex::scoped_lock lock(g_mutex);
    g_status = msg;

    if(!g_command){
        g_command.reset(new cob_hand_bridge::JointValues());
        g_command->position_cdeg = msg->joints.position_cdeg;
    }
    
    if(msg->status & msg->MASK_FINGER_READY){
        g_js.position.resize(msg->joints.position_cdeg.size());
        for(size_t i=0; i < msg->joints.position_cdeg.size(); ++i){
            g_js.position[i] = angles::from_degrees(msg->joints.position_cdeg[i]/100.0);
        }
        g_js.header.stamp = msg->stamp;
        g_js_pub.publish(g_js);
    }
}

bool initCallback(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res)
{
    boost::mutex::scoped_lock lock(g_mutex);
    
    ros::NodeHandle nh_priv("~");
    
    if(g_status->status == g_status->NOT_INITIALIZED) {
        lock.unlock();
        cob_hand_bridge::InitFinger srv;
        srv.request.port = nh_priv.param<std::string>("sdhx/port", "/dev/ttyACM0");
        srv.request.min_pwm0 = nh_priv.param("sdhx/min_pwm0", 0);
        srv.request.min_pwm1 = nh_priv.param("sdhx/min_pwm1", 0);
        srv.request.min_pwm0 = nh_priv.param("sdhx/min_pwm0", 0);
        srv.request.min_pwm1 = nh_priv.param("sdhx/max_pwm1", 0);

        if(!g_init_finger_client.call(srv)) return false;
        res.success = srv.response.success;
    }else{
        res.success = true;
        res.message = "already initialized";
    }

    return true;
}
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "cob_hand_bridge_node");
    
    ros::NodeHandle nh;
    ros::NodeHandle nh_d("driver");
    ros::NodeHandle nh_i("internal");
    ros::NodeHandle nh_priv("~");
    
    if(!nh_priv.getParam("sdhx/joint_names", g_js.name)){
        ROS_ERROR("Please provide joint names for SDHx");
        return 1;
    }
    
    g_js_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
    
    ros::Subscriber status_sub = nh_i.subscribe("status", 1, statusCallback);
    g_init_finger_client = nh_i.serviceClient<cob_hand_bridge::InitFinger>("init_finger");
    g_command_pub = nh.advertise<cob_hand_bridge::JointValues>("command", 1);

    ros::ServiceServer init_srv = nh_d.advertiseService("init", initCallback);
    
    ros::spin();
    return 0;
}
