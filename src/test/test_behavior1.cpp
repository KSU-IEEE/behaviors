#include "test/test_behavior1.h"
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(behaviors::test_behavior, nodelet::Nodelet);
namespace behaviors {
test_behavior::test_behavior() : base_behavior(){
    name_ = "tester";
}
test_behavior::~test_behavior(){}

void test_behavior::test1_cb(const std_msgs::String::ConstPtr& msg) {
    std::string msg1 = "TESTER: received message "+msg->data;
    std::cout<<msg1<<std::endl;
}

void test_behavior::printnum_cb(const std_msgs::Empty::ConstPtr& msg) {
    std::cout<<"TESTER has number: "<<num<<std::endl;
}

bool test_behavior::control_loop(){
    NODELET_INFO("TESTER: sending message to tester 2");
    std_msgs::String msg;
    msg.data = message;
    pub_message.publish(msg);
}

void test_behavior::set_params(){
    // example of getting a global param
    nh_.getParam("base_num", num);
    // example of getting a nodelet param
    nh_.getParam("tester/msg", message);
}
void test_behavior::nodelet_init(){
    sub_test2 = nh_.subscribe("tester2/message", 1000, &test_behavior::test1_cb, this);
    pub_message = nh_.advertise<std_msgs::String>("tester1/message", 1000);
    sub_printNum = nh_.subscribe("print_msg", 1000, &test_behavior::printnum_cb, this);
}
}