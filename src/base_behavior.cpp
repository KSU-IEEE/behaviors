#include <behaviors/base_behavior.h>
namespace behaviors {
void base_behavior::set_nh(ros::NodeHandle nh) {
    nh_ = nh;
}
ros::NodeHandle base_behavior::nh() {
    return nh_;
}
string base_behavior::node_name() {
    return name_;
}
bool base_behavior::in_control() {
    return in_control_;
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
    
    // grab params from launch files
    beh_.set_params();

    /*
    TODO
    Create something to create a publisher or subscriber 
    to to run `control_loop` when the state machine activates it
    */
    while(nh.ok()) {
        beh_.control_loop();
    }
}
} // namespace behaviors