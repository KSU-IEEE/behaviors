#include "test/test_behavior2.h"
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(behaviors::test_behavior2, nodelet::Nodelet);
namespace behaviors {
test_behavior2::test_behavior2() : base_behavior(){
    name_ = "tester2";
}
test_behavior2::~test_behavior2(){}

void test_behavior2::test2_cb(const std_msgs::String::ConstPtr& msg) {
    std::string msg1 = "TESTER2: received message "+msg->data;
    std::cout<<msg1<<std::endl;
}

void test_behavior2::printnum_cb(const std_msgs::Empty::ConstPtr& msg) {
    std::cout<<"TESTER2 has number: "<<num<<std::endl;
}

bool test_behavior2::control_loop(){
    NODELET_INFO("TESTER2: sending message to TESTER");
    std_msgs::String msg;
    msg.data = message;
    pub_message.publish(msg);
}

void test_behavior2::set_params(){
    // example of getting a global param
    nh_.getParam("base_num", num);
    // example of getting a nodelet param
    nh_.getParam("tester2/msg", message);
}
void test_behavior2::nodelet_init(){
    sub_test2 = nh_.subscribe("tester1/message", 1000, &test_behavior2::test2_cb, this);
    pub_message = nh_.advertise<std_msgs::String>("tester2/message", 1000);
    sub_printNum = nh_.subscribe("print_msg", 1000, &test_behavior2::printnum_cb, this);
}
}