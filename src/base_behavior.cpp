#include <base_behavior.h>
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

void base_behavior::activate_cb(const behaviors::target::ConstPtr& activate_val) {
    if(activate_val->activate.data && !in_control_) {
        in_control_ = true;
        set_goal(activate_val->goal.data);
        deviceThread_ = boost::shared_ptr< boost::thread > 
            (new boost::thread(boost::bind(&base_behavior::parent_control_loop, this)));
        return;
    } else if(!activate_val->activate.data && in_control_) {
        in_control_ = false;
    }
}

void base_behavior::print_msg(std::string msg) {
    std::cout<< name_ + ": " + msg <<std::endl;
}

bool base_behavior::control_loop(){
    std::string str = name_ + " in control";
    std::cout<<str<<std::endl;
    return false;
}
void base_behavior::set_params(){}
void base_behavior::nodelet_init(){}

void base_behavior::parent_control_loop(){
    bool success;
    while(in_control_) {
        success = control_loop();
        if(success) {
            std_msgs::Bool msg;
            msg.data = success;
            sm_pub.publish(msg);
        }
    }

}

void base_behavior::onInit() {
    // Create a ROS node handle
    NODELET_INFO("CREATING NODE");
    ros::NodeHandle nh;
    set_nh(nh);
    
    // initialize subscribers && grab params from launch files
    set_params();
    nodelet_init();

    // setup route to activate with sm 
    string topic_name = node_name() + "/activate";
    sm_sub = nh.subscribe(topic_name, 1000, &base_behavior::activate_cb, this);

    topic_name = node_name() + "/completed";
    sm_pub = nh.advertise<std_msgs::Bool>(topic_name, 1000);
}
} // namespace behaviors