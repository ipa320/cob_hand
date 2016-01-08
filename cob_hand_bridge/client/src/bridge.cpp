#include <ros/node_handle.h>

#include <cob_hand_bridge/InitPins.h>
#include <cob_hand_bridge/UpdatePins.h>
#include <cob_hand_bridge/Status.h>
#include <std_msgs/UInt8.h>

#include <boost/thread/thread.hpp>

#include "hardware.h"
#include "gpio.h"

ros::NodeHandle_<HandBridgeHardware> g_nh;

// GPIO
GPIO g_gpio;

using cob_hand_bridge::InitPins;
void handleInitPins(const InitPins::Request & req, InitPins::Response & res){
    res.success = g_gpio.init();

    for(size_t i = 0; i < req.input_pins_length; ++i)
        if(!g_gpio.setInput(req.input_pins[i]))
            res.success = false;

    for(size_t i = 0; i < req.output_pins_length; ++i)
        if(!g_gpio.setOutput(req.output_pins[i]))
            res.success = false;
}
ros::ServiceServer<InitPins::Request, InitPins::Response> g_srv_init_pins("init_pins",&handleInitPins);

using cob_hand_bridge::UpdatePins;
void handleUpdatePins(const UpdatePins::Request & req, UpdatePins::Response & res){
    res.success = g_gpio.setPins(req.set_pins) | g_gpio.clearPins(req.clear_pins);
    uint32_t state = g_gpio.getState();
    res.success = res.success && (state & req.set_pins) == req.set_pins && (state & req.clear_pins) == 0;
}
ros::ServiceServer<UpdatePins::Request, UpdatePins::Response> g_srv_update_pins("update_pins",&handleUpdatePins);

void handleClearPin(const std_msgs::UInt8& mgs){
    if(mgs.data < 32) g_gpio.clearPins(1<<mgs.data);
}
ros::Subscriber<std_msgs::UInt8> g_sub_clear_pin("clear_pin", handleClearPin );

void handleSetPin(const std_msgs::UInt8& mgs){
    if(mgs.data < 32) g_gpio.setPins(1<<mgs.data);
}
ros::Subscriber<std_msgs::UInt8> g_sub_set_pin("set_pin", handleSetPin );

cob_hand_bridge::Status g_status_msg;
ros::Publisher g_pub("status", &g_status_msg);

void step(){
    g_status_msg.stamp = g_nh.now();

    g_status_msg.status = g_status_msg.NOT_INITIALIZED;
    if(g_gpio.isInitialized()) g_status_msg.status |= g_status_msg.MASK_GPIO_READY;

    g_status_msg.pins = g_gpio.getState();
    g_pub.publish( &g_status_msg );
    g_nh.spinOnce();

    ++g_status_msg.seq;
}

int main(int argc, char** argv){

    double rate = 50;

    if(argc != 2 || argc != 3){
        std::cerr << "Usage: " << argv[0] << "port@baud [looprate]" << std::endl;
        exit(1);
    }
    if(argc == 3){
        rate = boost::lexical_cast<double>(argv[2]);
        if(rate <= 0.0){
            std::cerr << "Rate must be postive" << std::endl;
            exit(1);
        }
    }

    g_nh.initNode(argv[1]);

    g_nh.advertiseService(g_srv_init_pins);
    g_nh.advertiseService(g_srv_update_pins);
    g_nh.subscribe(g_sub_clear_pin);
    g_nh.subscribe(g_sub_set_pin);
    g_nh.advertise(g_pub);

    boost::chrono::duration<double> double_interval(1.0/rate);
    boost::chrono::milliseconds interval = boost::chrono::duration_cast<boost::chrono::milliseconds>(double_interval);
    boost::chrono::steady_clock::time_point next = boost::chrono::steady_clock::now();

    while(true) {
        step();
        next += interval;
        boost::this_thread::sleep_until(next);
    }

    return 0;
}
