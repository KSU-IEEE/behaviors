#include <behaviors/base_behavior.h>
namespace behaviors {
base_behavior::base_behavior() {}
base_behavior::base_behavior(string node_name) {
    set_node_name(node_name);
}
base_behavior::~base_behavior() {}

void base_behavior::set_nh(ros::NodeHandle nh) {
    nh_ = nh;
}
ros::NodeHandle base_behavior::nh() {
    return nh_;
}

string base_behavior::node_name() {
    return name_;
}
void base_behavior::set_node_name(string name) {
    name_ = name;
}

bool base_behavior::in_control() {
    return in_control_;
}

void base_behavior::activate_cb(const std_msgs::Bool::ConstPtr& activate) {
    in_control_ = activate->data;
    while(in_control_)control_loop();
}

void base_behavior::onInit() {
    // Create a ROS node handle
    ros::NodeHandle nh;
    set_nh(nh);
    
    // initialize subscribers && grab params from launch files
    nodelet_init();
    set_params();

    // setup route to activate with sm 
    string topic_name = node_name() + "/activate";
    ros::Subscriber sm_sub = nh.subscribe(topic_name, 1000, &base_behavior::activate_cb, this);
}
} // namespace behaviors