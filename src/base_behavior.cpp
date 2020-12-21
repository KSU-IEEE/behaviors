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
}

// to be called by each rosnode. set beh to the new behavior class 
// the MUST inherit from base_behavior
template<class beh>
int main(int argc, char* argv[])
{
    // initialize class 
    beh beh_ = new beh();
    // This must be called before anything else ROS-related
    ros::init(argc, argv, beh_.node_name());

    // Create a ROS node handle
    ros::NodeHandle nh;
    beh_.set_nh(nh);
    
    // initialize subscribers && grab params from launch files
    beh_.init();
    beh_.set_params();

    // setup route to activate with sm 
    string topic_name = beh_.node_name() + "/activate";
    ros::Subscriber sm_sub = nh.subscribe(topic_name, 1000, beh_.activate_cb);

    while(nh.ok()) {
        if (beh_.in_control()) beh_.control_loop();
    }
}
} // namespace behaviors