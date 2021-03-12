#include <pac-man-bot/return_to_start.h>

PLUGINLIB_EXPORT_CLASS(behaviors::pac_man_behs::return_to_start, nodelet::Nodelet);

namespace behaviors {
namespace pac_man_behs {
return_to_start::return_to_start() : base_behavior{"return_to_start"} {

}

return_to_start::~return_to_start() {

}


// override
bool return_to_start::control_loop() {
    if (!sent_) {
        // get msg ready
        behaviors::coordinate msg;
        msg.X = HOME_LOC.first;
        msg.Y = HOME_LOC.second;

        pub_goTo.publish(msg);

        sent_ = true;
        done_moving_ = false;

        // send arm to reset
        std_msgs::Bool arm_msg;
        arm_msg.data = true;
        pub_armReset.publish(arm_msg);
    }

    return sent_ && done_moving_;
}

void return_to_start::set_params() {

}

void return_to_start::nodelet_init() {
    sub_finishedMove = nh().subscribe("moveDone" , 1000, &return_to_start::finishedMove_cb, this);

    pub_goTo = nh().advertise<behaviors::coordinate>("moveTo", 1000);
    pub_armReset = nh().advertise<std_msgs::Bool>("/arm/reset", 1000);

}



} // pac_man_behs
} // behaviors