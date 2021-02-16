// includes
#include <pac-man-bot/init.h>
// plugin 
PLUGINLIB_EXPORT_CLASS(behaviors::pac_man_behs::init, nodelet::Nodelet);

// namespace
namespace behaviors {
namespace pac_man_behs {

init::init() : base_behavior {"init"} {
    // really just hve this for the above line
}

init::~init() {

}

// callbacks
void init::startSignal_cb(const std_msgs::Bool::ConstPtr& start) {
    // set begin
    begin_ = start->data;
    // start listening and set timer
    if (begin_) {
        std_msgs::Bool msg;
        msg.data = true;
        pubStartListen.publish(msg);

        start_time_ = std::chrono::system_clock::now();

        print_msg("Starting Timer");
    }
}

void init::ppLoc_cb (const std_msgs::Char::ConstPtr& loc) {
    power_loc_ = (loc->data == 'A' or loc->data == 'C') ? true : false;

    print_msg("Received loc: " + loc->data);
}

// timer function
bool init::stopwatch(int seconds) {
    std::chrono::duration<float> diff = 
        std::chrono::system_clock::now() - start_time_;

    // send status
    int secs = int(floor(diff.count()));
    if (secs % 5 == 0 && secs != last_msg_) {
        print_msg("has waited for " + std::to_string(secs) + " seconds");
        last_msg_ = secs;
    }
    return diff.count() >= seconds;
}

bool init::control_loop() {
    if (begin_) {
        time_out_ = stopwatch(THIRTY_SECS);
    }
    
    return begin_ && (power_loc_ || time_out_);
}

void init::set_params() {
// I don't think I need any params
}

void init::nodelet_init() {
    // init subs
    subStartSignal  = nh().subscribe("StartSignal", 1000, &init::startSignal_cb , this);
    subPPLoc        = nh().subscribe("PPLocation" , 1000, &init::ppLoc_cb       , this);

    // init pubs
    pubStartListen  = nh().advertise<std_msgs::Bool>("StartListen", 1000);
}

}// pac_man_behs
}// behaviors